classdef ContactorDecisionTest < matlab.unittest.TestCase
    % CI smoke/regression suite for the contactor command model.
    %
    % Current model interface:
    %   1  ReserveSwitch                boolean
    %   2  ReserveModeVehicleType       Enum: ReserveMode
    %   3  isLoadRequested              boolean
    %   4  isChargeRequested            boolean
    %   5  Contactor_looptime           uint32
    %   6  merlynEnable                 boolean
    %   7  ContactorCommandfromMerlyn   boolean
    %   8  vcuFrameRx                   boolean
    %   9  vcuLoadMissing               boolean
    %   10 vcuChargeMissing             boolean
    %   11 vcuChargeCommand             boolean
    %   12 vcuLoadCommand               boolean
    %   13 vcuDebounceCycle             uint8
    %   14 DisplaySOC                   int16
    %
    % ReserveMode mapping:
    %   RESERVE_MODE_OFF    = 0
    %   RESERVE_MODE_AUTO   = 1
    %   RESERVE_MODE_MANUAL = 2

    properties (Constant)
        Model = 'matlabmodel';
        Ts = 0.1;

        InNames = { ...
            'ReserveSwitch', ...
            'ReserveModeVehicleType', ...
            'isLoadRequested', ...
            'isChargeRequested', ...
            'Contactor_looptime', ...
            'merlynEnable', ...
            'ContactorCommandfromMerlyn', ...
            'vcuFrameRx', ...
            'vcuLoadMissing', ...
            'vcuChargeMissing', ...
            'vcuChargeCommand', ...
            'vcuLoadCommand', ...
            'vcuDebounceCycle', ...
            'DisplaySOC'};
    end

    methods (TestClassSetup)
        function loadModel(tc)

            here = fileparts(mfilename('fullpath'));
            addpath(fullfile(here, '..', 'models'));

            % ---------------------------------------------------------
            % ReserveMode
            %
            % GitHub Actions starts MATLAB with a clean environment.
            % Define ReserveMode only if it is not already available.
            % ---------------------------------------------------------
            try
                Simulink.findIntEnumType('ReserveMode');
                enumExists = true;
            catch
                enumExists = false;
            end

            if ~enumExists
                Simulink.defineIntEnumType( ...
                    'ReserveMode', ...
                    {'RESERVE_MODE_OFF', ...
                     'RESERVE_MODE_AUTO', ...
                     'RESERVE_MODE_MANUAL'}, ...
                    [0 1 2]);
            end

            % Load model after ReserveMode is available.
            load_system(tc.Model);

            % Enumeration external input cannot be interpolated.
            set_param( ...
                [tc.Model '/ReserveModeVehicleType'], ...
                'Interpolate', 'off');

            tc.addTeardown(@() close_system(tc.Model, 0));
        end
    end

    methods (Static)

        function b = baseline()

            % Baseline:
            %   ReserveMode = MANUAL (2)
            %   looptime = 100 ms
            %   debounce = 10 cycles
            %   VCU load command requested
            %
            % The enum is intentionally represented as numeric 2 here.
            % castToPort() converts it to ReserveMode.MANUAL.

            b = struct( ...
                'ReserveSwitch', false, ...
                'ReserveModeVehicleType', 2, ...
                'isLoadRequested', true, ...
                'isChargeRequested', false, ...
                'Contactor_looptime', uint32(100), ...
                'merlynEnable', false, ...
                'ContactorCommandfromMerlyn', false, ...
                'vcuFrameRx', true, ...
                'vcuLoadMissing', false, ...
                'vcuChargeMissing', false, ...
                'vcuChargeCommand', false, ...
                'vcuLoadCommand', true, ...
                'vcuDebounceCycle', uint8(10), ...
                'DisplaySOC', int16(50));

        end
    end

    methods (Access = private)

        function v = castToPort(tc, name, v)

            switch name

                case {'ReserveSwitch', ...
                      'isLoadRequested', ...
                      'isChargeRequested', ...
                      'merlynEnable', ...
                      'ContactorCommandfromMerlyn', ...
                      'vcuFrameRx', ...
                      'vcuLoadMissing', ...
                      'vcuChargeMissing', ...
                      'vcuChargeCommand', ...
                      'vcuLoadCommand'}

                    v = logical(v);

                case 'Contactor_looptime'

                    v = uint32(v);

                case 'vcuDebounceCycle'

                    v = uint8(v);

                case 'DisplaySOC'

                    v = int16(v);

                case 'ReserveModeVehicleType'

                    % Convert numeric mapping explicitly:
                    %
                    %   0 -> OFF
                    %   1 -> AUTO
                    %   2 -> MANUAL

                    v = arrayfun( ...
                        @(x) tc.toReserveMode(x), ...
                        v);

                otherwise

                    error( ...
                        'ContactorDecisionTest:UnknownInput', ...
                        'Unknown model input: %s', ...
                        name);
            end
        end

        function e = toReserveMode(~, x)

            switch double(x)

                case 0
                    e = ReserveMode.RESERVE_MODE_OFF;

                case 1
                    e = ReserveMode.RESERVE_MODE_AUTO;

                case 2
                    e = ReserveMode.RESERVE_MODE_MANUAL;

                otherwise
                    error( ...
                        'ContactorDecisionTest:InvalidReserveMode', ...
                        ['Invalid ReserveMode value %g. ', ...
                         'Expected 0 (OFF), 1 (AUTO), or 2 (MANUAL).'], ...
                        double(x));
            end
        end

        function y = simulate(tc, overrides, n)

            s = tc.baseline();

            fields = fieldnames(overrides);

            for k = 1:numel(fields)
                s.(fields{k}) = overrides.(fields{k});
            end

            t = (0:n-1)' * tc.Ts;

            ds = Simulink.SimulationData.Dataset;

            for k = 1:numel(tc.InNames)

                name = tc.InNames{k};

                v = s.(name);
                v = v(:);

                if isscalar(v)
                    v = repmat(v, n, 1);
                end

                tc.verifyEqual( ...
                    numel(v), ...
                    n, ...
                    sprintf('%s must contain %d samples.', ...
                    name, n));

                v = tc.castToPort(name, v);

                ts = timeseries(v, t, 'Name', name);

                % Zero-order hold for all external inputs.
                ts = setinterpmethod(ts, 'zoh');

                ds = ds.addElement(ts, name);
            end

            in = Simulink.SimulationInput(tc.Model);

            in = in.setModelParameter( ...
                'SolverType', 'Fixed-step', ...
                'Solver', 'FixedStepDiscrete', ...
                'FixedStep', num2str(tc.Ts), ...
                'StopTime', num2str((n-1) * tc.Ts), ...
                'SaveOutput', 'on', ...
                'OutputSaveName', 'yout', ...
                'SaveFormat', 'Dataset', ...
                'ReturnWorkspaceOutputs', 'on');

            in = in.setExternalInput(ds);

            out = sim(in);

            y = double( ...
                squeeze(out.yout{1}.Values.Data));

            tc.verifyEqual( ...
                numel(y), ...
                n, ...
                'Model did not return the expected number of output samples.');

        end
    end

    methods (Test)

        function testModelRunsBaseline(tc)

            % Basic CI smoke test:
            % model loads, accepts all inputs and produces output.

            y = tc.simulate(struct(), 20);

            tc.verifyEqual(numel(y), 20);

        end

        function testMerlynOverride(tc)

            % Merlyn enabled + command = 1
            % should command the contactor.

            y = tc.simulate(struct( ...
                'merlynEnable', true, ...
                'ContactorCommandfromMerlyn', true, ...
                'isLoadRequested', false, ...
                'vcuLoadCommand', false), 10);

            tc.verifyTrue( ...
                all(y ~= 0), ...
                'Merlyn command = 1 should command the contactor.');

        end

        function testMerlynOpenCommand(tc)

            % Merlyn enabled + command = 0
            % should force the contactor open.

            y = tc.simulate(struct( ...
                'merlynEnable', true, ...
                'ContactorCommandfromMerlyn', false), 10);

            tc.verifyTrue( ...
                all(y == 0), ...
                'Merlyn command = 0 should open the contactor.');

        end

        function testReserveModeEnumInput(tc)

            % Exercise all three ReserveMode values:
            %
            %   0 = OFF
            %   1 = AUTO
            %   2 = MANUAL

            modes = [0 1 2];

            for k = 1:numel(modes)

                y = tc.simulate( ...
                    struct( ...
                        'ReserveModeVehicleType', modes(k)), ...
                    5);

                tc.verifyEqual( ...
                    numel(y), ...
                    5, ...
                    'ReserveMode value %d did not simulate correctly.', ...
                    modes(k));

            end
        end
    end
end
