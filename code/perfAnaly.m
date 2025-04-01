function varargout = perfAnaly(varargin)
% PERFANALY MATLAB code for perfAnaly.fig
%      PERFANALY, by itself, creates a new PERFANALY or raises the existing
%      singleton*.
%
%      H = PERFANALY returns the handle to a new PERFANALY or the handle to
%      the existing singleton*.
%
%      PERFANALY('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PERFANALY.M with the given input arguments.
%
%      PERFANALY('Property','Value',...) creates a new PERFANALY or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before perfAnaly_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to perfAnaly_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help perfAnaly

% Last Modified by GUIDE v2.5 26-Dec-2019 11:43:50

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @perfAnaly_OpeningFcn, ...
                   'gui_OutputFcn',  @perfAnaly_OutputFcn, ...
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


% --- Executes just before perfAnaly is made visible.
function perfAnaly_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to perfAnaly (see VARARGIN)

% Choose default command line output for perfAnaly
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes perfAnaly wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = perfAnaly_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on mouse press over axes background.
function axes3_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function ZOOM_Callback(hObject, eventdata, handles)
% hObject    handle to ZOOM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
