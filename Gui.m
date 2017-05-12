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

% Last Modified by GUIDE v2.5 11-May-2017 14:18:07

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
s = Slam.getInstance();
s.runSimulation();
% s.show();
% s.showRobot();

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


% --- Executes on button press in forward.
function forward_Callback(hObject, eventdata, handles)
% hObject    handle to forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

s = Slam.getInstance();
s.deleteOldRobot();
dir = deg2rad(s.robot.theta);
s.change(0.2*cos(dir),0.2*sin(dir),0);
s.showRobot();

% --- Executes on button press in rotateCCW.
function rotateCCW_Callback(hObject, eventdata, handles)
% hObject    handle to rotateCCW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

s = Slam.getInstance();
s.deleteOldRobot();
s.change(0,0,10);
s.showRobot();

% --- Executes on button press in rotateCW.
function rotateCW_Callback(hObject, eventdata, handles)
% hObject    handle to rotateCW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

s = Slam.getInstance();
s.deleteOldRobot();
s.change(0,0,-10);
s.showRobot();

% --- Executes on button press in backward.
function backward_Callback(hObject, eventdata, handles)
% hObject    handle to backward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

s = Slam.getInstance();
s.deleteOldRobot();
dir = deg2rad(s.robot.theta);
s.change(-0.2*cos(dir),-0.2*sin(dir),0);
s.showRobot();

% --- Executes on key press with focus on figure1 or any of its controls.
function figure1_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
keyPressed = eventdata.Key;
switch keyPressed
case 'uparrow'
    forward_Callback(handles.forward,[],handles);
case 'downarrow'
    backward_Callback(handles.backward,[],handles);
case 'leftarrow'
    rotateCCW_Callback(handles.rotateCCW,[],handles);
case 'rightarrow'
    rotateCW_Callback(handles.rotateCW,[],handles);
otherwise
    disp('not an arrow key pressed');
end

% --- Executes on button press in showScan.
function showScan_Callback(hObject, eventdata, handles)
% hObject    handle to showScan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of showScan

s = Slam.getInstance();
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
set(handles.showScan, 'Value', 0);
if get(hObject,'Value') == 1
    s.setScan = false;
    s.deleteScan();
end