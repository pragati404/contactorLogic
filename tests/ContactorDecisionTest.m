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


    %% ================================================================
    %  MODEL SETUP
    %  ================================================================

    methods (TestClassSetup)

        function loadModel(tc)

            % Get folder containing this test file
            here = fileparts(mfilename('fullpath'));

            % Model is stored one folder above tests/
            modelsFolder = fullfile(here, '..', 'models');

            % Add model folder to MATLAB path
            addpath(modelsFolder);

            % Full model path
            modelFile = fullfile( ...
                modelsFolder, ...
                [tc.Model '.slx']);

            % Check that model exists
            if ~isfile(modelFile)

                error( ...
                    'ContactorDecisionTest:MissingModel', ...
                    'Model not found: %s', ...
                    modelFile);

            end

            % Load model
            load_system(modelFile);

            % Close model after all tests
            tc.addTeardown(@() close_system(tc.Model, 0));

        end

    end


    %% ================================================================
    %  BASELINE INPUT VALUES
    %  ================================================================

    methods (Static)

        function s = baseline()

            % All values are explicitly typed to match the model
            % root-level Inport datatypes.

            s = struct( ...

                % boolean
                'ReserveSwitch', ...
                logical(false), ...

                % int8
                % 0 = OFF
                % 1 = AUTO
                % 2 = MANUAL
                'ReserveModeVehicleType', ...
                int8(2), ...

                % boolean
                'isLoadRequested', ...
                logical(true), ...

                % boolean
                'isChargeRequested', ...
                logical(false), ...

                % uint32
                'Contactor_looptime', ...
                uint32(100), ...

                % boolean
                'merlynEnable', ...
                logical(false), ...

                % boolean
                'ContactorCommandfromMerlyn', ...
                logical(false), ...

                % boolean
                'vcuFrameRx', ...
                logical(true), ...

                % boolean
                'vcuLoadMissing', ...
                logical(false), ...

                % boolean
                'vcuChargeMissing', ...
                logical(false), ...

                % boolean
                'vcuChargeCommand', ...
                logical(false), ...

                % boolean
                'vcuLoadCommand', ...
                logical(true), ...

                % uint8
                'vcuDebounceCycle', ...
                uint8(10), ...

                % int16
                'DisplaySOC', ...
                int16(50));

        end

    end


    %% ================================================================
    %  INPUT DATATYPE CONVERSION
    %  ================================================================

    methods (Access = private)

        function v = castToPort(~, name, v)

            switch name

                % ----------------------------------------------------
                % BOOLEAN INPUTS
                % ----------------------------------------------------

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


                % ----------------------------------------------------
                % RESERVE MODE
                %
                % 0 = OFF
                % 1 = AUTO
                % 2 = MANUAL
                % ----------------------------------------------------

                case 'ReserveModeVehicleType'

                    v = int8(v);


                % ----------------------------------------------------
                % UINT32 INPUT
                % ----------------------------------------------------

                case 'Contactor_looptime'

                    v = uint32(v);


                % ----------------------------------------------------
                % UINT8 INPUT
                % ----------------------------------------------------

                case 'vcuDebounceCycle'

                    v = uint8(v);


                % ----------------------------------------------------
                % INT16 INPUT
                % ----------------------------------------------------

                case 'DisplaySOC'

                    v = int16(v);


                otherwise

                    error( ...
                        'ContactorDecisionTest:UnknownInput', ...
                        'Unknown model input: %s', ...
                        name);

            end

        end


        %% ============================================================
        %  SIMULATION
        %  ============================================================

        function y = simulate(tc, overrides, n)

            % --------------------------------------------------------
            % Start with baseline values
            % --------------------------------------------------------

            s = tc.baseline();


            % --------------------------------------------------------
            % Apply testcase-specific overrides
            % --------------------------------------------------------

            fields = fieldnames(overrides);

            for k = 1:numel(fields)

                s.(fields{k}) = overrides.(fields{k});

            end


            % --------------------------------------------------------
            % Generate monotonically increasing simulation time
            % --------------------------------------------------------

            t = (0:n-1)' * tc.Ts;


            % --------------------------------------------------------
            % Create Simulink Dataset
            % --------------------------------------------------------

            ds = Simulink.SimulationData.Dataset;


            % --------------------------------------------------------
            % Add every root-level input
            % --------------------------------------------------------

            for k = 1:numel(tc.InNames)

                name = tc.InNames{k};

                v = s.(name);

                % Make column vector
                v = v(:);


                % If only one value is supplied,
                % hold that value for the complete simulation.

                if isscalar(v)

                    v = repmat(v, n, 1);

                end


                % Make sure number of samples is correct

                tc.verifyEqual( ...
                    numel(v), ...
                    n, ...
                    sprintf( ...
                        '%s must contain %d samples.', ...
                        name, ...
                        n));


                % Convert to exact model port datatype

                v = tc.castToPort(name, v);


                % Create timeseries

                ts = timeseries( ...
                    v, ...
                    t, ...
                    'Name', ...
                    name);


                % Zero-order hold for discrete inputs

                ts = setinterpmethod( ...
                    ts, ...
                    'zoh');


                % Add to Dataset

                ds = ds.addElement( ...
                    ts, ...
                    name);

            end


            % --------------------------------------------------------
            % Configure simulation
            % --------------------------------------------------------

            in = Simulink.SimulationInput(tc.Model);

            in = in.setModelParameter( ...

                'SolverType', ...
                'Fixed-step', ...

                'Solver', ...
                'FixedStepDiscrete', ...

                'FixedStep', ...
                num2str(tc.Ts), ...

                'StopTime', ...
                num2str((n-1) * tc.Ts), ...

                'SaveOutput', ...
                'on', ...

                'OutputSaveName', ...
                'yout', ...

                'SaveFormat', ...
                'Dataset', ...

                'ReturnWorkspaceOutputs', ...
                'on');


            % --------------------------------------------------------
            % Apply external inputs
            % --------------------------------------------------------

            in = in.setExternalInput(ds);


            % --------------------------------------------------------
            % Run model
            % --------------------------------------------------------

            out = sim(in);


            % --------------------------------------------------------
            % Extract first output
            % --------------------------------------------------------

            y = double( ...
                squeeze( ...
                    out.yout{1}.Values.Data));


            % --------------------------------------------------------
            % Verify output length
            % --------------------------------------------------------

            tc.verifyEqual( ...
                numel(y), ...
                n, ...
                'Model did not return expected output samples.');

        end

    end


    %% ================================================================
    %  TEST CASES
    %  ================================================================

    methods (Test)


        %% ------------------------------------------------------------
        %  TEST 1
        %  Basic model execution
        %  ------------------------------------------------------------

        function testModelRunsBaseline(tc)

            y = tc.simulate( ...
                struct(), ...
                20);

            tc.verifyEqual( ...
                numel(y), ...
                20, ...
                'Baseline simulation did not produce 20 samples.');

        end


        %% ------------------------------------------------------------
        %  TEST 2
        %  Merlyn command = ON
        %  ------------------------------------------------------------

        function testMerlynOverride(tc)

            y = tc.simulate( ...
                struct( ...
                    'merlynEnable', ...
                    logical(true), ...

                    'ContactorCommandfromMerlyn', ...
                    logical(true), ...

                    'isLoadRequested', ...
                    logical(false), ...

                    'vcuLoadCommand', ...
                    logical(false)), ...
                10);


            tc.verifyTrue( ...
                all(y ~= 0), ...
                ['Merlyn command = 1 should command ', ...
                 'the contactor.']);

        end


        %% ------------------------------------------------------------
        %  TEST 3
        %  Merlyn command = OFF
        %  ------------------------------------------------------------

        function testMerlynOpenCommand(tc)

            y = tc.simulate( ...
                struct( ...
                    'merlynEnable', ...
                    logical(true), ...

                    'ContactorCommandfromMerlyn', ...
                    logical(false)), ...
                10);


            tc.verifyTrue( ...
                all(y == 0), ...
                ['Merlyn command = 0 should open ', ...
                 'the contactor.']);

        end


        %% ------------------------------------------------------------
        %  TEST 4
        %  ReserveMode input values
        %
        %  0 = OFF
        %  1 = AUTO
        %  2 = MANUAL
        %  ------------------------------------------------------------

        function testReserveModeInput(tc)


            % --------------------------------------------------------
            % OFF
            % --------------------------------------------------------

            y = tc.simulate( ...
                struct( ...
                    'ReserveModeVehicleType', ...
                    int8(0)), ...
                5);

            tc.verifyEqual( ...
                numel(y), ...
                5, ...
                'ReserveMode OFF did not simulate.');


            % --------------------------------------------------------
            % AUTO
            % --------------------------------------------------------

            y = tc.simulate( ...
                struct( ...
                    'ReserveModeVehicleType', ...
                    int8(1)), ...
                5);

            tc.verifyEqual( ...
                numel(y), ...
                5, ...
                'ReserveMode AUTO did not simulate.');


            % --------------------------------------------------------
            % MANUAL
            % --------------------------------------------------------

            y = tc.simulate( ...
                struct( ...
                    'ReserveModeVehicleType', ...
                    int8(2)), ...
                5);

            tc.verifyEqual( ...
                numel(y), ...
                5, ...
                'ReserveMode MANUAL did not simulate.');

        end

    end

end
