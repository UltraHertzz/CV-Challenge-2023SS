function varargout = CV_GUI(varargin)
% CV_GUI MATLAB code for CV_GUI.fig
%      CV_GUI, by itself, creates a new CV_GUI or raises the existing
%      singleton*.
%
%      H = CV_GUI returns the handle to a new CV_GUI or the handle to
%      the existing singleton*.
%
%      CV_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CV_GUI.M with the given input arguments.
%
%      CV_GUI('Property','Value',...) creates a new CV_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before CV_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to CV_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CV_GUI

% Last Modified by GUIDE v2.5 20-Jun-2023 11:02:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CV_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @CV_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before CV_GUI is made visible.
function CV_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CV_GUI (see VARARGIN)

% Choose default command line output for CV_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
set(handles.figure1, 'Color', [0.94, 0.94, 0.94]);
% UIWAIT makes CV_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CV_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filenames, path] = uigetfile({'*.jpg;*.png'},'Select Image Files','MultiSelect','on');

if isequal(filenames,0)
    disp('User selected Cancel');
else
    % Create cell array of full file paths
    if iscell(filenames)
        fullpaths = cellfun(@(name) fullfile(path, name), filenames, 'UniformOutput', false);
    else
        fullpaths = {fullfile(path, filenames)};
    end
    % Store full file paths in handles structure
    handles.selectedImages = fullpaths;
    guidata(hObject, handles); % Save handles structure
end


handles = guidata(hObject);
selectedImages = handles.selectedImages;

numImages = numel(selectedImages);
numCols = 2;  % adjust as needed
numRows = ceil(numImages / numCols);
for k = 1:numImages
    subplot(numRows, numCols, k, 'Parent', handles.panel1);
    imshow(imread(selectedImages{k}));
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);
imds = imageDatastore(handles.selectedImages);


fx = str2double(get(handles.edit1, 'String'));
fy = str2double(get(handles.edit2, 'String'));
cx = str2double(get(handles.edit3, 'String'));
cy = str2double(get(handles.edit4, 'String'));
img_width = str2double(get(handles.edit5, 'String'));
img_height = str2double(get(handles.edit6, 'String'));


FocalLength = [fx, fy];
PrincipalPoint = [cx, cy];
ImageSize = [img_height,img_width];


RadialDistortion = [0, 0];
TangentialDistortion = [0, 0];
Skew = 0;

% 创建相机内参数
cameraIntrinsicsObj = cameraIntrinsics(FocalLength, PrincipalPoint, ImageSize, 'RadialDistortion', RadialDistortion, 'TangentialDistortion', TangentialDistortion, 'Skew', Skew);

save('cameraIntrinsics.mat', 'cameraIntrinsicsObj');

images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
    I = readimage(imds, i);
    images{i} = im2gray(I);
end

title('Input Image Sequence');

%%
data = load('cameraIntrinsics.mat');
intrinsics = data.cameraIntrinsicsObj;

%%
% Get intrinsic parameters of the camera
I = undistortImage(images{1}, intrinsics); 
% Detect features. Increasing 'NumOctaves' helps detect large-scale
% features in high-resolution images. Use an ROI to eliminate spurious
% features around the edges of the image.
border = 50;
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];
prevPoints   = detectSURFFeatures(I, MetricThreshold=400,NumOctaves=8, ROI=roi);

% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(I, prevPoints, Upright=true);

% Create an empty imageviewset object to manage the data associated with each
% view.
vSet = imageviewset;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, rigidtform3d, Points=prevPoints);
%%
for i = 2:numel(images)
    % Undistort the current image.
    I = undistortImage(images{i}, intrinsics);
    
    % Detect, extract and match features.
    currPoints   = detectSURFFeatures(I,MetricThreshold=1000, NumOctaves=8, ROI=roi);
    currFeatures = extractFeatures(I, currPoints, Upright=true);    
    indexPairs   = matchFeatures(prevFeatures, currFeatures, ...
        MaxRatio=0.7, Unique=true);
    
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
 


    [relPose, inlierIdx] = helperEstimateRelativePose(...
        matchedPoints1, matchedPoints2, intrinsics);
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1).AbsolutePose;
        
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    currPose = rigidtform3d(prevPose.A*relPose.A);
    
    % Add the current view to the view set.
    vSet = addView(vSet, i, currPose, Points=currPoints);

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, relPose, Matches=indexPairs(inlierIdx,:));
    
    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, intrinsics);
    
    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, intrinsics, FixedViewId=1, ...
        PointsUndistorted=true,MaxIterations=2000, RelativeTolerance= 1e-7, AbsoluteTolerance= 1e-22);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
end
%%
% Display camera poses.
set(handles.figure1, 'Color', [1, 1, 1]);
camPoses = poses(vSet);
axes(handles.axes2);
plotCamera(camPoses, Size=0.2);
hold on

% Exclude noisy 3-D points.
goodIdx = (reprojectionErrors < 5);
xyzPoints = xyzPoints(goodIdx, :);

% Display the 3-D points.
pcshow(xyzPoints, VerticalAxis='y', VerticalAxisDir='down', MarkerSize= 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.AbsolutePose(1).Translation;
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Refined Camera Poses');

set(handles.figure1, 'Color', [0.94, 0.94, 0.94]);
%%
% Read and undistort the first image
I = undistortImage(images{1}, intrinsics); 

% Detect corners in the first image.
prevPoints = detectMinEigenFeatures(I, MinQuality=0.001);

% Create the point tracker object to track the points across views.
tracker = vision.PointTracker(MaxBidirectionalError=1, NumPyramidLevels=6);

% Initialize the point tracker.
prevPoints = prevPoints.Location;
initialize(tracker, prevPoints, I);

% Store the dense points in the view set.

vSet = updateConnection(vSet, 1, 2, Matches=zeros(0, 2));
vSet = updateView(vSet, 1, Points=prevPoints);

% Track the points across all views.
for i = 2:numel(images)
    % Read and undistort the current image.
    I = undistortImage(images{i}, intrinsics); 
    
    % Track the points.
    [currPoints, validIdx] = step(tracker, I);
    
    % Clear the old matches between the points.
    if i < numel(images)
        vSet = updateConnection(vSet, i, i+1, Matches=zeros(0, 2));
    end
    vSet = updateView(vSet, i, Points=currPoints);
    
    % Store the point matches in the view set.
    matches = repmat((1:size(prevPoints, 1))', [1, 2]);
    matches = matches(validIdx, :);        
    vSet = updateConnection(vSet, i-1, i, Matches=matches);
end

% Find point tracks across all views.
tracks = findTracks(vSet);

% Find point tracks across all views.
camPoses = poses(vSet);

% Triangulate initial locations for the 3-D world points.
xyzPoints = triangulateMultiview(tracks, camPoses,...
    intrinsics);

% Refine the 3-D world points and camera poses.
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
    xyzPoints, tracks, camPoses, intrinsics, FixedViewId=1, ...
    PointsUndistorted=true);

%%

% Display the refined camera poses.
axes(handles.axes3);
plotCamera(camPoses, Size=0.2);
hold on

% Exclude noisy 3-D world points.
goodIdx = (reprojectionErrors < 5);

% Display the dense 3-D world points.
pcshow(xyzPoints(goodIdx, :), VerticalAxis='y', VerticalAxisDir='down', MarkerSize=45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.AbsolutePose(1).Translation;
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Dense Reconstruction');

set(handles.figure1, 'Color', [0.94, 0.94, 0.94]);





% --- Executes during object creation, after setting all properties.
function panel1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to panel1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes3



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnReadOrInput.
function btnReadOrInput_Callback(hObject, eventdata, handles)
% hObject    handle to btnReadOrInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Ask the user to choose the method
% Ask the user to choose the method
choice = questdlg('Would you like to read the camera parameters from a file or input them manually?', ...
    'Choose Method', ...
    'Read from file', 'Input manually', 'Cancel', 'Cancel');

    switch choice
        case 'Read from file'
            % Let the user select the .txt file
            [filename, path] = uigetfile('*.txt', 'Select the cameras.txt file');
            
            % When the user cancels file selection, filename will be '0'
            if isequal(filename, '0')
                disp('User cancelled file selection.');
            else
                direction = fullfile(path, filename);  % Concatenate the full path of the file
                camInfo = cameraReader(direction);
            
            % Set the camera parameters to the edit boxes
            set(handles.edit5, 'String', num2str(camInfo(1)));
            set(handles.edit6, 'String', num2str(camInfo(2)));
            set(handles.edit1, 'String', num2str(camInfo(3)));
            set(handles.edit2, 'String', num2str(camInfo(4)));
            set(handles.edit3, 'String', num2str(camInfo(5)));
            set(handles.edit4, 'String', num2str(camInfo(6)));

            % Create and save cameraIntrinsics object
            FocalLength = [camInfo(3), camInfo(4)];
            PrincipalPoint = [camInfo(5), camInfo(6)];
            ImageSize = [camInfo(2), camInfo(1)];
            RadialDistortion = [0, 0];
            TangentialDistortion = [0, 0];
            Skew = 0;
            cameraIntrinsicsObj = cameraIntrinsics(FocalLength, PrincipalPoint, ImageSize, 'RadialDistortion', RadialDistortion, 'TangentialDistortion', TangentialDistortion, 'Skew', Skew);
            save('cameraIntrinsics.mat', 'cameraIntrinsicsObj');  
            end

case 'Input manually'
    prompt = {'Enter the width:', 'Enter the height:', 'Enter fx:', 'Enter fy:', 'Enter cx:', 'Enter cy:'};
    dlgtitle = 'Input';
    dims = [1 35];
    definput = {'', '', '', '', '', ''};
    answer = inputdlg(prompt, dlgtitle, dims, definput);
    
    if ~isempty(answer)  % if input is given
        camInfo = cellfun(@str2num, answer, 'UniformOutput', false);
        camInfo = cell2mat(camInfo);
        
        % Set the camera parameters to the edit boxes
        set(handles.edit5, 'String', num2str(camInfo(1)));
        set(handles.edit6, 'String', num2str(camInfo(2)));
        set(handles.edit1, 'String', num2str(camInfo(3)));
        set(handles.edit2, 'String', num2str(camInfo(4)));
        set(handles.edit3, 'String', num2str(camInfo(5)));
        set(handles.edit4, 'String', num2str(camInfo(6)));
        
        % Create and save cameraIntrinsics object
        FocalLength = [camInfo(3), camInfo(4)];
        PrincipalPoint = [camInfo(5), camInfo(6)];
        ImageSize = [camInfo(2), camInfo(1)];
        RadialDistortion = [0, 0];
        TangentialDistortion = [0, 0];
        Skew = 0;
        cameraIntrinsicsObj = cameraIntrinsics(FocalLength, PrincipalPoint, ImageSize, 'RadialDistortion', RadialDistortion, 'TangentialDistortion', TangentialDistortion, 'Skew', Skew);
        save('cameraIntrinsics.mat', 'cameraIntrinsicsObj');  
    end
    end