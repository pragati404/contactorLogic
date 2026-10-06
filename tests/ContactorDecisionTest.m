classdef ContactorDecisionTest < matlab.unittest.TestCase

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


    %% Model setup
    methods (TestClassSetup)

        function loadModel(tc)

            % Get model folder relative to this test file
            here = fileparts(mfilename('fullpath'));
            modelsFolder = fullfile(here, '..', 'models');

            addpath(modelsFolder);

            % Load model
            load_system(fullfile(modelsFolder, [tc.Model '.slx']));

            % Close model after all tests
            tc.addTeardown(@() close_system(tc.Model, 0));

        end

    end


    %% Baseline input values
    methods (Static)

        function s = baseline()

            s = struct( ...
                'ReserveSwitch', false, ...
                'ReserveModeVehicleType', int8(2), ...   % MANUAL
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


    %% Input datatype conversion
    methods (Access = private)

        function v = castToPort(~, name, v)

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

                case 'ReserveModeVehicleType'

                    % 0 = OFF
                    % 1 = AUTO
                    % 2 = MANUAL
                    v = int8(v);

                case 'Contactor_looptime'

                    v = uint32(v);

                case 'vcuDebounceCycle'

                    v = uint8(v);

                case 'DisplaySOC'

                    v = int16(v);

                otherwise

                    error( ...
                        'ContactorDecisionTest:UnknownInput', ...
                        'Unknown model input: %s', name);

            end

        end


        %% Run simulation
        function y = simulate(tc, overrides, n)

            % Start with baseline values
            s = tc.baseline();

            % Apply testcase-specific overrides
            fields = fieldnames(overrides);

            for k = 1:numel(fields)
                s.(fields{k}) = overrides.(fields{k});
            end

            % Simulation time
            t = (0:n-1)' * tc.Ts;

            % Create external input dataset
            ds = Simulink.SimulationData.Dataset;

            for k = 1:numel(tc.InNames)

                name = tc.InNames{k};

                v = s.(name);
                v = v(:);

                % Repeat scalar input for every timestep
                if isscalar(v)
                    v = repmat(v, n, 1);
                end

                tc.verifyEqual( ...
                    numel(v), ...
                    n, ...
                    sprintf('%s must contain %d samples.', name, n));

                % Convert to model input datatype
                v = tc.castToPort(name, v);

                % Zero-order hold
                ts = timeseries(v, t, 'Name', name);
                ts = setinterpmethod(ts, 'zoh');

                ds = ds.addElement(ts, name);

            end


            %% Simulation configuration

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

            % Apply external inputs
            in = in.setExternalInput(ds);

            % Run simulation
            out = sim(in);

            % Extract first output
            y = double(squeeze(out.yout{1}.Values.Data));

            tc.verifyEqual( ...
                numel(y), ...
                n, ...
                'Model did not return expected output samples.');

        end

    end


    %% Test cases
    methods (Test)

        %% 1. Basic model execution
        function testModelRunsBaseline(tc)

            y = tc.simulate( ...
                struct(), ...
                20);

            tc.verifyEqual( ...
                numel(y), ...
                20);

        end


        %% 2. Merlyn controls contactor
        function testMerlynOverride(tc)

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


        %% 3. Merlyn opens contactor
        function testMerlynOpenCommand(tc)

            y = tc.simulate( ...
                struct( ...
                    'merlynEnable', true, ...
                    'ContactorCommandfromMerlyn', false), ...
                10);

            tc.verifyTrue( ...
                all(y == 0), ...
                'Merlyn command = 0 should open contactor.');

        end


        %% 4. Verify ReserveMode values
        function testReserveModeInput(tc)

            % 0 = OFF
            y = tc.simulate( ...
                struct( ...
                    'ReserveModeVehicleType', int8(0)), ...
                5);

            tc.verifyEqual( ...
                numel(y), ...
                5, ...
                'ReserveMode OFF did not simulate.');


            % 1 = AUTO
            y = tc.simulate( ...
                struct( ...
                    'ReserveModeVehicleType', int8(1)), ...
                5);

            tc.verifyEqual( ...
                numel(y), ...
                5, ...
                'ReserveMode AUTO did not simulate.');


            % 2 = MANUAL
            y = tc.simulate( ...
                struct( ...
                    'ReserveModeVehicleType', int8(2)), ...
                5);

            tc.verifyEqual( ...
                numel(y), ...
                5, ...
                'ReserveMode MANUAL did not simulate.');

        end

    end

end
