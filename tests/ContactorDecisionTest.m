classdef ContactorDecisionTest < matlab.unittest.TestCase
    % CI smoke test for the contactor command model.
    %
    % ReserveMode:
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

            modelsFolder = fullfile(here, '..', 'models');

            addpath(modelsFolder);

            % ---------------------------------------------------------
            % LOAD THE DATA DICTIONARY
            % ---------------------------------------------------------
            dictFile = fullfile(modelsFolder, 'forContactor.sldd');

            if ~isfile(dictFile)
                error( ...
                    'ContactorDecisionTest:MissingDictionary', ...
                    'Data dictionary not found: %s', dictFile);
            end

            Simulink.data.dictionary.open(dictFile);

            % ---------------------------------------------------------
            % LOAD MODEL
            % ---------------------------------------------------------
            load_system(tc.Model);

            % ---------------------------------------------------------
            % ENUM ROOT INPUT
            %
            % External enum input data cannot be interpolated.
            % ---------------------------------------------------------
            set_param( ...
                [tc.Model '/ReserveModeVehicleType'], ...
                'Interpolate', ...
                'off');

            tc.addTeardown(@() close_system(tc.Model, 0));

        end
    end

    methods (Static)

        function b = baseline()

            % Baseline:
            %
            % ReserveMode = MANUAL = 2
            % Contactor loop time = 100 ms
            % Debounce = 10 cycles

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

                case { ...
                        'ReserveSwitch', ...
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

                    % -------------------------------------------------
                    % Convert numeric test value to ReserveMode enum.
                    %
                    % 0 = OFF
                    % 1 = AUTO
                    % 2 = MANUAL
                    %
                    % The enum definition is supplied by the data
                    % dictionary loaded during TestClassSetup.
                    % -------------------------------------------------

                    v = arrayfun( ...
                        @(x) tc.convertReserveMode(x), ...
                        v);

                otherwise

                    error( ...
                        'ContactorDecisionTest:UnknownInput', ...
                        'Unknown model input: %s', ...
                        name);

            end
        end

        function e = convertReserveMode(~, x)

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
                         'Expected 0, 1 or 2.'], ...
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
                    sprintf( ...
                    '%s must contain %d samples.', ...
                    name, ...
                    n));

                v = tc.castToPort(name, v);

                ts = timeseries( ...
                    v, ...
                    t, ...
                    'Name', ...
                    name);

                ts = setinterpmethod(ts, 'zoh');

                ds = ds.addElement( ...
                    ts, ...
                    name);

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
                squeeze( ...
                out.yout{1}.Values.Data));

            tc.verifyEqual( ...
                numel(y), ...
                n, ...
                'Model did not return expected output samples.');

        end
    end

    methods (Test)

        function testModelRunsBaseline(tc)

            % Basic smoke test:
            % model loads and runs with all 14 inputs.

            y = tc.simulate( ...
                struct(), ...
                20);

            tc.verifyEqual( ...
                numel(y), ...
                20);

        end

        function testMerlynOverride(tc)

            % Merlyn enabled + command = 1
            % should command contactor.

            y = tc.simulate( ...
                struct( ...
                    'merlynEnable', true, ...
                    'ContactorCommandfromMerlyn', true, ...
                    'isLoadRequested', false, ...
                    'vcuLoadCommand', false), ...
                10);

            tc.verifyTrue( ...
                all(y ~= 0), ...
                'Merlyn command = 1 should command contactor.');

        end

        function testMerlynOpenCommand(tc)

            % Merlyn enabled + command = 0
            % should force contactor open.

            y = tc.simulate( ...
                struct( ...
                    'merlynEnable', true, ...
                    'ContactorCommandfromMerlyn', false), ...
                10);

            tc.verifyTrue( ...
                all(y == 0), ...
                'Merlyn command = 0 should open contactor.');

        end

        function testReserveModeEnumInput(tc)

            % Test all three ReserveMode values:
            %
            % 0 = OFF
            % 1 = AUTO
            % 2 = MANUAL

            modes = [0 1 2];

            for k = 1:numel(modes)

                y = tc.simulate( ...
                    struct( ...
                        'ReserveModeVehicleType', modes(k)), ...
                    5);

                tc.verifyEqual( ...
                    numel(y), ...
                    5, ...
                    'ReserveMode value %d did not simulate.', ...
                    modes(k));

            end

        end
    end
end
