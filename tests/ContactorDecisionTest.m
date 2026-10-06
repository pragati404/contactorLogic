classdef ContactorDecisionTest < matlab.unittest.TestCase
    % Small CI smoke/regression suite for the current contactor model.
    %
    % Current model interface (matlabmodel.slx):
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
    % This is intentionally a SMALL CI suite. Once this passes, expand the
    % full 22-test suite.

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

            load_system(tc.Model);

            % Enum external input data cannot be interpolated.
            % Disable interpolation on this root-level Inport once,
            % instead of modifying SimulationInput on every test.
            set_param([tc.Model '/ReserveModeVehicleType'], ...
                'Interpolate', 'off');

            tc.addTeardown(@() close_system(tc.Model, 0));
        end
    end

    methods (Static)
        function b = baseline()
            % Baseline chosen to request a closed contactor after boot/debounce.
            b = struct( ...
                'ReserveSwitch', false, ...
                'ReserveModeVehicleType', 0, ...
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
                'vcuDebounceCycle', uint8(5), ...
                'DisplaySOC', int16(50));
        end
    end

    methods (Access = private)

        function v = castToPort(tc, name, v)
            switch name
                case {'ReserveSwitch', ...
                      'isLoadRequested', 'isChargeRequested', ...
                      'merlynEnable', 'ContactorCommandfromMerlyn', ...
                      'vcuFrameRx', 'vcuLoadMissing', ...
                      'vcuChargeMissing', 'vcuChargeCommand', ...
                      'vcuLoadCommand'}
                    v = logical(v);

                case 'Contactor_looptime'
                    v = uint32(v);

                case 'vcuDebounceCycle'
                    v = uint8(v);

                case 'DisplaySOC'
                    v = int16(v);

                case 'ReserveModeVehicleType'
                    % The model has already been loaded, so ReserveMode
                    % should be available in the MATLAB/Simulink environment.
                    %
                    % Use the actual enum member from the model rather than
                    % assuming its underlying numeric value.
                    try
                        v = arrayfun(@(x) ...
                            tc.enumFromValue(x), v);
                    catch ME
                        error('ContactorDecisionTest:EnumConversion', ...
                            ['Could not convert ReserveModeVehicleType to ', ...
                             'ReserveMode enum. Original error:\n%s'], ...
                            ME.message);
                    end

                otherwise
                    error('ContactorDecisionTest:UnknownInput', ...
                        'Unknown model input: %s', name);
            end
        end

        function e = enumFromValue(~, x)
            % The model has already been loaded. Resolve the actual enum
            % class registered by Simulink, then construct the enum using
            % its underlying numeric value.
            try
                % Try to get enum info with just the name
                defaultEnum = Simulink.data.getEnumTypeInfo('ReserveMode', 'DefaultValue');
                enumClass = class(defaultEnum);
            catch
                % If that fails, search in the model's data dictionary or workspace
                % Get all enum types defined in the model
                enumInfo = Simulink.data.getEnumTypeInfo();
                % Find ReserveMode in the list
                idx = strcmp({enumInfo.Name}, 'ReserveMode');
                if any(idx)
                    defaultEnum = enumInfo(idx).DefaultValue;
                    enumClass = class(defaultEnum);
                else
                    error('ReserveMode enum not found in model');
                end
            end
            e = feval(enumClass, x);
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

                tc.verifyEqual(numel(v), n, ...
                    sprintf('%s must contain %d samples.', name, n));

                v = tc.castToPort(name, v);

                ts = timeseries(v, t, 'Name', name);
                ts = setinterpmethod(ts, 'zoh');
                ds = ds.addElement(ts, name);
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

            tc.verifyEqual(numel(y), n, ...
                'Model did not return the expected number of output samples.');
        end
    end

    methods (Test)

        function testModelRunsBaseline(tc)
            % 20 steps = only 2 seconds of simulation.
            y = tc.simulate(struct(), 20);

            % Before bootDone (> 1000 ms), contactor should remain open.
            tc.verifyTrue(all(y(1:10) == 0), ...
                'Baseline should remain open during boot delay.');

            % After boot, baseline should request closure.
            tc.verifyTrue(all(y(11:20) ~= 0), ...
                'Baseline should close after boot.');
        end

        function testMerlynOverride(tc)
            % Merlyn should command the output directly.
            y = tc.simulate(struct( ...
                'merlynEnable', true, ...
                'ContactorCommandfromMerlyn', true, ...
                'isLoadRequested', false, ...
                'vcuLoadCommand', false), 10);

            tc.verifyTrue(all(y ~= 0), ...
                'Merlyn command = 1 should command the contactor.');
        end

        function testMerlynOpenCommand(tc)
            % Merlyn command = 0 should force the contactor open.
            y = tc.simulate(struct( ...
                'merlynEnable', true, ...
                'ContactorCommandfromMerlyn', false), 10);

            tc.verifyTrue(all(y == 0), ...
                'Merlyn command = 0 should open the contactor.');
        end

        function testEnumInputLoads(tc)
            % Exercise the ReserveMode enum root Inport. The purpose of this
            % smoke test is to prove that enum external input data can be
            % loaded and simulated without interpolation errors.
            y = tc.simulate(struct( ...
                'ReserveModeVehicleType', 0), 10);

            tc.verifyEqual(numel(y), 10);
        end
    end
end
