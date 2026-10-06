classdef ContactorDecisionTest < matlab.unittest.TestCase
    % Regression tests for the contactor decision model.
    %
    % Assumptions (change here if your model differs):
    %   - The model file is models/<Model>.slx with 14 root Inports and 1 Outport.
    %   - Inports are in the order listed in InNames.
    %   - Step time is 0.1 s and looptime_ms = 100.
    %   - Step k of a simulation is at time (k-1)*0.1 s.
    %   - vcuAge < 5000 ms means the VCU is recent. bootDone when uptime > 1000 ms.
    %   - vcuDeb flips on the Nth consecutive mismatching step (N = vcuDebounceCycle).

    properties (Constant)
        Model = 'matlabmodel';   % CHANGE to your model name (without .slx)
        Ts    = 0.1;
        MODE_OFF = 0;
        MODE_AUTO = 1;
        MODE_MANUAL = 2;
        InNames = {'isLoadRequested','isChargeRequested','looptime_ms','vcuFrameRx', ...
                   'vcuLoadmissing','vcuChargemissing','vcuChargeCommand','vcuLoadCommand', ...
                   'vcuDebounceCycle','VehicleModeType','Soc','ReserveSwitch', ...
                   'merlynEnable','ContactorCommandfromMerlyn'};
    end

    methods (TestClassSetup)
        function loadModel(tc)
            here = fileparts(mfilename('fullpath'));
            addpath(fullfile(here, '..', 'models'));
            load_system(tc.Model);
            tc.addTeardown(@() close_system(tc.Model, 0));
        end
    end

    methods (Static)
        function b = baseline()
            % Inputs that give ContactorCommand = 1 once boot and debounce are done.
            b = struct( ...
                'isLoadRequested', 1, 'isChargeRequested', 0, 'looptime_ms', 100, ...
                'vcuFrameRx', 1, 'vcuLoadmissing', 0, 'vcuChargemissing', 0, ...
                'vcuChargeCommand', 0, 'vcuLoadCommand', 1, 'vcuDebounceCycle', 10, ...
                'VehicleModeType', 1, 'Soc', 50, 'ReserveSwitch', 0, ...
                'merlynEnable', 0, 'ContactorCommandfromMerlyn', 0);
        end
    end

    methods (Access = private)

        function v = castToPort(tc, nm, v)
            % Cast each test input to the datatype configured on the
            % corresponding root-level Inport in the model.
            %
            % Current model datatypes:
            %   boolean : isLoadRequested, isChargeRequested, vcuFrameRx,
            %             vcuLoadmissing, vcuChargemissing,
            %             vcuChargeCommand, vcuLoadCommand,
            %             ReserveSwitch, merlynEnable,
            %             ContactorCommandfromMerlyn
            %   uint32  : looptime_ms
            %   uint16  : vcuDebounceCycle, Soc
            %   Enum ReserveMode : VehicleModeType
            %
            % ReserveMode values:
            %   0 = OFF, 1 = AUTO, 2 = MANUAL

            switch nm
                case { ...
                        'isLoadRequested', ...
                        'isChargeRequested', ...
                        'vcuFrameRx', ...
                        'vcuLoadmissing', ...
                        'vcuChargemissing', ...
                        'vcuChargeCommand', ...
                        'vcuLoadCommand', ...
                        'ReserveSwitch', ...
                        'merlynEnable', ...
                        'ContactorCommandfromMerlyn'}
                    v = logical(v);

                case 'looptime_ms'
                    v = uint32(v);

                case {'vcuDebounceCycle', 'Soc'}
                    v = uint16(v);

                case 'VehicleModeType'
                    % VehicleModeType is the ReserveMode enum:
                    % 0 = OFF, 1 = AUTO, 2 = MANUAL.
                    %
                    % Do not call ReserveMode(...) directly here. In CI,
                    % the enum may be owned by the model/data dictionary
                    % and therefore not be directly visible as a MATLAB
                    % class name even though Simulink knows the type.
                    %
                    % First obtain the actual enum object/type from
                    % Simulink. If the type is not registered yet, define
                    % the known ReserveMode type for the test environment.
                    try
                        defaultEnum = Simulink.data.getEnumTypeInfo( ...
                            'ReserveMode', 'DefaultValue');
                        enumClass = class(defaultEnum);
                    catch
                        if isempty(Simulink.findIntEnumType('ReserveMode'))
                            Simulink.defineIntEnumType( ...
                                'ReserveMode', ...
                                {'OFF', 'AUTO', 'MANUAL'}, ...
                                [0 1 2]);
                        end
                        enumClass = 'ReserveMode';
                    end

                    % Convert sample-by-sample so vector inputs are
                    % supported reliably by Dataset/timeseries input.
                    v = arrayfun(@(x) feval(enumClass, x), v);

                otherwise
                    % Fallback for any future input that is not listed
                    % above. This also avoids treating "Inherit: auto" as
                    % a real datatype.
                    blk = [tc.Model '/' nm];

                    try
                        dt = get_param(blk, 'OutDataTypeStr');
                    catch
                        dt = 'double';
                    end

                    switch lower(strtrim(dt))
                        case {'boolean', 'bool'}
                            v = logical(v);

                        case {'uint8','uint16','uint32','uint64', ...
                              'int8','int16','int32','int64', ...
                              'single','double'}
                            v = cast(v, dt);

                        otherwise
                            v = double(v);
                    end
            end
        end

        function y = simulate(tc, over, n)
            % over: struct of overrides.
            % Scalar = constant, vector (length n) = per step.

            s = tc.baseline();

            f = fieldnames(over);
            for i = 1:numel(f)
                s.(f{i}) = over.(f{i});
            end

            t = (0:n-1)' * tc.Ts;
            ds = Simulink.SimulationData.Dataset;

            for i = 1:numel(tc.InNames)
                nm = tc.InNames{i};
                v = s.(nm);
                v = v(:);

                if isscalar(v)
                    v = repmat(v, n, 1);
                end

                tc.assertEqual( ...
                    numel(v), n, ...
                    sprintf('Input %s has %d samples, expected %d.', ...
                    nm, numel(v), n));

                v = tc.castToPort(nm, v);

                ts = timeseries(v, t, 'Name', nm);
                ts = setinterpmethod(ts, 'zoh');
                ds = ds.addElement(ts, nm);
            end

            in = Simulink.SimulationInput(tc.Model);

            in = in.setModelParameter( ...
                'SolverType', 'Fixed-step', ...
                'Solver', 'FixedStepDiscrete', ...
                'FixedStep', num2str(tc.Ts), ...
                'StopTime', num2str((n-1)*tc.Ts), ...
                'SaveOutput', 'on', ...
                'OutputSaveName', 'yout', ...
                'SaveFormat', 'Dataset', ...
                'ReturnWorkspaceOutputs', 'on');

            in = in.setExternalInput(ds);
            out = sim(in);

            y = double(squeeze(out.yout{1}.Values.Data));

            tc.assertEqual( ...
                numel(y), n, ...
                'Unexpected number of output samples.');
        end

        function expectRange(tc, y, a, b, val, msg)
            tc.verifyTrue( ...
                all(y(a:b) == val), ...
                sprintf('%s: expected %d on steps %d..%d, got [%s]', ...
                msg, val, a, b, num2str(y(a:b)')));
        end

    end

    methods (Test)

        % ---------- Steady-state truth table (one simulation, 3072 combinations) ----------
        function testSteadyStateTruthTable(tc)
            seg = 12;    % steps per combination, > debounce cycles so state settles
            warm = 15;   % warm-up steps so bootDone = 1
            [iL,iC,vL,vC,mL,mC,md,so,rs,me,mc] = ndgrid( ...
                0:1, 0:1, 0:1, 0:1, 0:1, 0:1, [0 1 2], [0 50], 0:1, 0:1, 0:1);
            nC = numel(iL);
            b = tc.baseline();
            rp = @(x, base) [repmat(base, warm, 1); repelem(x(:), seg)];
            n = warm + nC*seg;
            over = struct( ...
                'isLoadRequested',            rp(iL, b.isLoadRequested), ...
                'isChargeRequested',          rp(iC, b.isChargeRequested), ...
                'vcuLoadCommand',             rp(vL, b.vcuLoadCommand), ...
                'vcuChargeCommand',           rp(vC, b.vcuChargeCommand), ...
                'vcuLoadmissing',             rp(mL, b.vcuLoadmissing), ...
                'vcuChargemissing',           rp(mC, b.vcuChargemissing), ...
                'VehicleModeType',            rp(md, b.VehicleModeType), ...
                'Soc',                        rp(so, b.Soc), ...
                'ReserveSwitch',              rp(rs, b.ReserveSwitch), ...
                'merlynEnable',               rp(me, b.merlynEnable), ...
                'ContactorCommandfromMerlyn', rp(mc, b.ContactorCommandfromMerlyn));
            y = tc.simulate(over, n);
            actual = y(warm + (1:nC)*seg) ~= 0;

            L = @(x) logical(x(:));
            rec  = ~(L(mL) | L(mC));
            deb  = L(vL) | L(vC);
            req  = L(iL) | L(iC);
            base = req & (deb | ~rec);
            chg  = L(vC) | L(iC);
            blk  = (md(:) == tc.MODE_MANUAL) & (so(:) == 0) & ~chg & ~L(rs);
            expd = (L(me) & L(mc)) | (~L(me) & base & ~blk);

            bad = find(actual ~= expd);
            msg = '';
            if ~isempty(bad)
                k = bad(1);
                msg = sprintf(['%d of %d combinations differ. First: isLoad=%d isChg=%d vcuLoad=%d vcuChg=%d ' ...
                    'loadMissing=%d chgMissing=%d mode=%d soc=%d rsvSw=%d merlynEn=%d merlynCmd=%d ' ...
                    '-> expected %d, got %d'], numel(bad), nC, iL(k), iC(k), vL(k), vC(k), mL(k), mC(k), ...
                    md(k), so(k), rs(k), me(k), mc(k), expd(k), actual(k));
            end
            tc.verifyEmpty(bad, msg);
        end

        % ---------- Boot ----------
        function testBootDelayWithFallbackFlag(tc)
            y = tc.simulate(struct('vcuLoadmissing', 1, 'vcuLoadCommand', 0), 30);
            tc.expectRange(y, 1, 10, 0, 'Boot not done');
            tc.expectRange(y, 11, 30, 1, 'Boot done, fallback follows bmsReq');
        end

        function testBootDelaySlowLoopTime(tc)
            y = tc.simulate(struct('looptime_ms', 500, 'vcuLoadmissing', 1, 'vcuLoadCommand', 0), 10);
            tc.expectRange(y, 1, 2, 0, 'uptime 500 and 1000 ms is not > 1000');
            tc.expectRange(y, 3, 10, 1, 'uptime 1500 ms');
        end

        function testUptimeSaturatesWithoutWrapping(tc)
            y = tc.simulate(struct('vcuLoadmissing', 1, 'vcuLoadCommand', 0), 800);
            tc.expectRange(y, 11, 800, 1, 'Output after 80 s');
        end

        % ---------- VCU alive timer ----------
        function testPowerUpWithoutFrameIsFallback(tc)
            % If the age timer started at 0 instead of 65535, output would stay 0 for ~5 s.
            y = tc.simulate(struct('vcuFrameRx', 0, 'vcuLoadCommand', 0), 800);
            tc.expectRange(y, 1, 10, 0, 'Boot not done');
            tc.expectRange(y, 11, 800, 1, 'No frame ever, fallback, and age does not wrap');
        end

        function testVcuTimeoutBoundary(tc)
            frame = [ones(30,1); zeros(70,1)];
            y = tc.simulate(struct('vcuFrameRx', frame, 'vcuLoadCommand', 0), 100);
            tc.expectRange(y, 1, 79, 0, 'VCU recent but VCU does not ask');
            tc.expectRange(y, 80, 100, 1, 'VCU timed out, fallback follows bmsReq');
        end

        function testTimeoutHasNoEffectWhenVcuAsks(tc)
            frame = [ones(30,1); zeros(70,1)];
            y = tc.simulate(struct('vcuFrameRx', frame), 100);
            tc.expectRange(y, 1, 10, 0, 'Boot and debounce');
            tc.expectRange(y, 11, 100, 1, 'Closed before and after timeout');
        end

        function testRecoversWhenFramesReturn(tc)
            frame = [ones(30,1); zeros(70,1); ones(50,1)];
            y = tc.simulate(struct('vcuFrameRx', frame, 'vcuLoadCommand', 0), 150);
            tc.expectRange(y, 80, 100, 1, 'Fallback');
            tc.expectRange(y, 101, 150, 0, 'Frames back, VCU does not ask');
        end

        function testMissingFlagForcesFallbackAndRecovers(tc)
            flag = [zeros(20,1); ones(20,1); zeros(10,1)];
            y = tc.simulate(struct('vcuLoadmissing', flag, 'vcuLoadCommand', 0), 50);
            tc.expectRange(y, 11, 20, 0, 'Flag clear');
            tc.expectRange(y, 21, 40, 1, 'Flag set');
            tc.expectRange(y, 41, 50, 0, 'Flag cleared again');
        end

        % ---------- Debounce ----------
        function testDebounceRiseOnTenthStep(tc)
            vL = [zeros(20,1); ones(30,1)];
            y = tc.simulate(struct('vcuLoadCommand', vL), 50);
            tc.expectRange(y, 1, 29, 0, 'Before 10th consecutive high');
            tc.expectRange(y, 30, 50, 1, 'From 10th consecutive high');
        end

        function testDebounceRejectsShortHigh(tc)
            vL = [zeros(20,1); ones(9,1); zeros(21,1)];
            y = tc.simulate(struct('vcuLoadCommand', vL), 50);
            tc.expectRange(y, 1, 50, 0, '9-step high pulse');
        end

        function testDebounceRejectsShortLow(tc)
            vL = [ones(20,1); zeros(9,1); ones(21,1)];
            y = tc.simulate(struct('vcuLoadCommand', vL), 50);
            tc.expectRange(y, 1, 9, 0, 'Before first debounce');
            tc.expectRange(y, 11, 50, 1, '9-step low glitch ignored');
        end

        function testDebounceFallOnTenthStep(tc)
            vL = [ones(30,1); zeros(30,1)];
            y = tc.simulate(struct('vcuLoadCommand', vL), 60);
            tc.expectRange(y, 11, 39, 1, 'Before 10th consecutive low');
            tc.expectRange(y, 40, 60, 0, 'From 10th consecutive low');
        end

        function testDebounceCycleParameter(tc)
            vL = [zeros(20,1); ones(30,1)];
            y = tc.simulate(struct('vcuLoadCommand', vL, 'vcuDebounceCycle', 5), 50);
            tc.expectRange(y, 1, 24, 0, 'Before 5th high');
            tc.expectRange(y, 25, 50, 1, 'From 5th high');
        end

        function testSwapLoadAndChargeCommandsNoGlitch(tc)
            vL = [ones(15,1); zeros(35,1)];
            vC = [zeros(15,1); ones(35,1)];
            y = tc.simulate(struct('vcuLoadCommand', vL, 'vcuChargeCommand', vC), 50);
            tc.expectRange(y, 11, 50, 1, 'OR of load and charge commands');
        end

        % ---------- Reserve mode (dynamic) ----------
        function testReserveBlocksWhenSocReachesZero(tc)
            soc = [50*ones(40,1); zeros(20,1)];
            y = tc.simulate(struct('VehicleModeType', tc.MODE_MANUAL, 'Soc', soc), 60);
            tc.expectRange(y, 11, 40, 1, 'Soc > 0');
            tc.expectRange(y, 41, 60, 0, 'Soc = 0 in manual mode');
        end

        function testReserveSwitchReleasesBlock(tc)
            rs = [zeros(40,1); ones(20,1)];
            y = tc.simulate(struct('VehicleModeType', tc.MODE_MANUAL, 'Soc', 0, 'ReserveSwitch', rs), 60);
            tc.expectRange(y, 1, 40, 0, 'Blocked');
            tc.expectRange(y, 41, 60, 1, 'Reserve switch pressed');
        end

        function testChargeRequestReleasesBlock(tc)
            iC = [zeros(40,1); ones(20,1)];
            y = tc.simulate(struct('VehicleModeType', tc.MODE_MANUAL, 'Soc', 0, 'isChargeRequested', iC), 60);
            tc.expectRange(y, 1, 40, 0, 'Blocked');
            tc.expectRange(y, 41, 60, 1, 'GPIO charge request lifts the block');
        end

        % ---------- Merlyn ----------
        function testMerlynOverridesEverythingFromFirstStep(tc)
            o = struct('merlynEnable', 1, 'ContactorCommandfromMerlyn', 1, 'isLoadRequested', 0, ...
                'vcuLoadCommand', 0, 'VehicleModeType', tc.MODE_MANUAL, 'Soc', 0, 'vcuFrameRx', 0);
            y = tc.simulate(o, 20);
            tc.expectRange(y, 1, 20, 1, 'Merlyn command 1');
        end

        function testMerlynCommandZeroForcesOpen(tc)
            y = tc.simulate(struct('merlynEnable', 1, 'ContactorCommandfromMerlyn', 0), 30);
            tc.expectRange(y, 1, 30, 0, 'Merlyn command 0');
        end

        function testMerlynDisableReturnsToNormalPath(tc)
            me = [ones(20,1); zeros(30,1)];
            y = tc.simulate(struct('merlynEnable', me, 'ContactorCommandfromMerlyn', 1, 'isLoadRequested', 0), 50);
            tc.expectRange(y, 1, 20, 1, 'Merlyn enabled');
            tc.expectRange(y, 21, 50, 0, 'Merlyn off, bmsReq = 0');
        end

        function testMerlynCommandIgnoredWhenDisabled(tc)
            y = tc.simulate(struct('merlynEnable', 0, 'ContactorCommandfromMerlyn', 1, 'isLoadRequested', 0), 30);
            tc.expectRange(y, 1, 30, 0, 'Merlyn disabled');
        end
    end
end
