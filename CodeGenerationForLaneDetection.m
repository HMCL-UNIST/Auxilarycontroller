%% Verify GPU Environment
% To verify that the compilers and libraries necessary for running this
% example are set up correctly, use the
% <docid:gpucoder_ref#mw_0957d820-192f-400a-8045-0bb746a75278
% coder.checkGpuInstall> function.
envCfg = coder.gpuEnvConfig('host');
envCfg.DeepLibTarget = 'cudnn';
envCfg.DeepCodegen = 1;
envCfg.Quiet = 1;
coder.checkGpuInstall(envCfg);

%% Lane and Vehicle Detection Simulink Model
open_system('LaneDetection');
%% Run the Simulation
% Open Configuration Parameters dialog box.
%
% In *Simulation Target* pane, select *GPU acceleration*. In the
% *Deep Learning* group, select the target library as *cuDNN*.
set_param(bdroot,'GPUAcceleration','on');
set_param(bdroot,'SimDLTargetLibrary','cudnn');
set_param(bdroot,'TargetLang','C++');
set_param(bdroot,'GenerateGPUCode','CUDA');
set_param(bdroot,'DLTargetLibrary','cudnn');
%%
set_param('LaneDetection', 'SimulationMode', 'Normal');
sim('LaneDetection');
%%
%
% <<../CodeGenerationSettings.png>>
%
% In the subcategory *Libraries* of the *Code Generation > GPU Code* pane,
% enable *cuBLAS*, *cuSOLVER* and *cuFFT*.
set_param(bdroot,'GPUcuBLAS','on');
set_param(bdroot,'GPUcuSOLVER','on');
set_param(bdroot,'GPUcuFFT','on');
%%
status = evalc("rtwbuild('laneAndVehicleDetection')");
%% Generated CUDA Code
% The subfolder named |laneAndVehicleDetection_ert_rtw| contains the generated 
% C++ codes corresponding to the different blocks in the Simulink model and the
% specific operations being performed in those blocks. For example, the
% file |trainedLaneNet0_laneAndVehicleDetection0.h| contains the C++ class which 
% contains attributes and member functions representing the 
% pretrained lane detection network. 
%
% <<../trainedLaneNetCode.JPG>>
%
% Similarly, the file |yolov2ResNet50VehicleExample0_laneAndVehicleDetection0.h|
% contains the C++ class representing the pretrained YOLO v2 detection
% network. 
%
% <<../yolov2ResNet50VehicleExampleCode.JPG>>
