function varargout = figaxes(varargin)
% FIGAXES MATLAB code for figaxes.fig
%      FIGAXES, by itself, creates a new FIGAXES or raises the existing
%      singleton*.
%
%      H = FIGAXES returns the handle to a new FIGAXES or the handle to
%      the existing singleton*.
%
%      FIGAXES('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FIGAXES.M with the given input arguments.
%
%      FIGAXES('Property','Value',...) creates a new FIGAXES or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before figaxes_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to figaxes_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help figaxes

% Last Modified by GUIDE v2.5 24-Dec-2019 14:30:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 0;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @figaxes_OpeningFcn, ...
                   'gui_OutputFcn',  @figaxes_OutputFcn, ...
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


% --- Executes just before figaxes is made visible.
function figaxes_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to figaxes (see VARARGIN)

% Choose default command line output for figaxes
handles.output = hObject;
handles.cur1=0;
handles.cur2=0;
handles.cur3=0;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes figaxes wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = figaxes_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1

handles.cur1=get(hObject,'Value');
guidata(hObject, handles);


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2

handles.cur2=get(hObject,'Value');
guidata(hObject, handles);

% --- Executes on button press in checkbox3.
function checkbox3_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox3

handles.cur3=get(hObject,'Value');
guidata(hObject, handles);

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global tz rz yz uz;
cla(handles.axes1,'reset');
axes(handles.axes1)

if handles.cur1
    plot(tz,rz,'r','LineWidth',2)
    hold on;
end
if handles.cur2
    plot(tz,yz,'b','LineWidth',2)
    hold on;
end
if handles.cur3
    plot(tz,uz,'g')
    hold on;
end                 %Êä³öÏìÓ¦Í¼Ïñ
grid on;
xlabel('time')
ylabel('output')
