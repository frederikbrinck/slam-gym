function varargout = Gui(varargin)
% GUI MATLAB code for Gui.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the rotateCCW, property value pairs are
%      applied to the GUI before Gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Gui

% Last Modified by GUIDE v2.5 14-May-2017 22:17:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Gui_OpeningFcn, ...
                   'gui_OutputFcn',  @Gui_OutputFcn, ...
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


% --- Executes just before Gui is made visible.
function Gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Gui (see VARARGIN)

% Choose default command line output for Gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
set(handles.algMenu, 'Value', 1);
set(handles.envMenu, 'Value', 1);
set(handles.quitText,'String','');
cla;
Slam.initialize();

% UIWAIT makes Gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on key press with focus on figure1 or any of its controls.
function figure1_WindowKeyPressFcn(hObject, eventdata, handles)
    s = Slam.getInstance();
    % hObject    handle to figure1 (see GCBO)
    % eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
    %	Key: name of the key that was pressed, in lower case
    %	Character: character interpretation of the key(s) that was pressed
    %	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
    % handles    structure with handles and user data (see GUIDATA)
    keyPressed = eventdata.Key;
    switch keyPressed
        case 'uparrow'
            s.up = true;
        case 'downarrow'
            s.down = true;
        case 'leftarrow'
            s.left = true;
        case 'rightarrow'
            s.right = true;
    end
    

% --- Executes on button press in showScan.
function showScan_Callback(hObject, eventdata, handles)
% hObject    handle to showScan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of showScan

s = Slam.getInstance();
if get(hObject,'Value') == 0
    set(handles.hideScan, 'Value', 1);
    hideScan_Callback(handles.hideScan, [], handles); 
    return;
end
set(handles.hideScan, 'Value', 0);
if get(hObject,'Value') == 1
    s.setScan = true;
    s.showScan();
end

% --- Executes on button press in hideScan.
function hideScan_Callback(hObject, eventdata, handles)
% hObject    handle to hideScan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of hideScan

s = Slam.getInstance();
if get(hObject,'Value') == 0
    set(handles.showScan, 'Value', 1);
    showScan_Callback(handles.showScan, [], handles); 
    return;
end
set(handles.showScan, 'Value', 0);
if get(hObject,'Value') == 1
    s.setScan = false;
    s.deleteScan();
end

% --- Executes on selection change in envMenu.
function envMenu_Callback(hObject, eventdata, handles)
% hObject    handle to envMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns envMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from envMenu
contents = cellstr(get(hObject,'String'));
s = Slam.getInstance();
if s.startFlag == false
    file = strcat('environments/',contents{get(hObject,'Value')});
    a = Environment;
    a = a.readFile(file);
    s.env = a;
    s.robot.sensing.env = a;
    s.robot.x = a.start(1);
    s.robot.y = a.start(2);
end

% --- Executes during object creation, after setting all properties.
function envMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to envMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of start
s = Slam.getInstance();
if get(hObject,'Value') == true
    set(hObject,'String','Quit');
    set(handles.quitText,'String','Quitting may be a bit slow when using EKF');
    set(handles.envMenu,'enable','off');
    set(handles.algMenu,'enable','off');
    set(handles.hideScan,'enable','on');
    set(handles.showScan,'enable','on');
    set(handles.showScan, 'Value', 0);
    set(handles.hideScan, 'Value', 1);
    cla;
    s.startFlag = true;
    s.startSimulation();
    s.runSimulation();
else
    set(hObject,'String','Start');
    set(handles.quitText,'String','');
    set(handles.envMenu,'enable','on');
    set(handles.algMenu,'enable','on');
    set(handles.hideScan,'enable','off');
    set(handles.showScan,'enable','off');
    set(handles.showScan, 'Value', 0);
    set(handles.hideScan, 'Value', 1);
    s.startFlag = false;
    s.stopSimulation();
    cla;
end

% --- Executes on selection change in algMenu.
function algMenu_Callback(hObject, eventdata, handles)
% hObject    handle to algMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns algMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from algMenu
s = Slam.getInstance();
contents = cellstr(get(hObject,'String'));
str = contents{get(hObject,'Value')};
if strcmp(str,'Using EKF') == true
    s.usingEkf = true;
else
    s.usingEkf = false;
end

% --- Executes during object creation, after setting all properties.
function algMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to algMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key release with focus on figure1 and none of its controls.
function figure1_KeyReleaseFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was released, in lower case
%	Character: character interpretation of the key(s) that was released
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) released
% handles    structure with handles and user data (see GUIDATA)

s = Slam.getInstance();
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
keyPressed = eventdata.Key;
switch keyPressed
    case 'uparrow'
        s.up = false;
    case 'downarrow'
        s.down = false;
    case 'leftarrow'
        s.left = false;
    case 'rightarrow'
        s.right = false;
end
