function varargout = pidgui(varargin)
% PIDGUI MATLA B code for pidgui.fig
%      PIDGUI, by itself, creates a new PIDGUI or raises the existing
%      singleton*.
%
%      H = PIDGUI returns the handle to a new PIDGUI or the handle to
%      the existing singleton*.
%
%      PIDGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PIDGUI.M with the given input arguments.
%
%      PIDGUI('Property','Value',...) creates a new PIDGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before pidgui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to pidgui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help pidgui

% Last Modified by GUIDE v2.5 26-Dec-2019 11:39:40

% Begin initialization code - DO NOT EDIT
gui_Singleton = 0;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @pidgui_OpeningFcn, ...
    'gui_OutputFcn',  @pidgui_OutputFcn, ...
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

% --- Executes just before pidgui is made visible.
function pidgui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to pidgui (see VARARGIN)

% Choose default command line output for pidgui
handles.output = hObject;

% I=imread('H:\matlab_design\logo.jpg');     %д��logoͼƬ������
% javaImage=im2java(I);                      %ת����java����
% newIcon=javax.swing.ImageIcon(javaImage);  %
% figFrame=get(handles.figure1,'JavaFrame'); %ȡ��Figure��JavaFrame��
% figFrame.setFigureIcon(newIcon);           %�޸�ͼ��

%�����䴫�ݱ�����ʼ��
handles.kp=0;                
handles.ki=0;
handles.kd=0;                %PID������ʼ��
handles.num=[];
handles.den=[];              %���ݺ������ӷ�ĸϵ����ʼ��
handles.s=0;                 %�ź�ѡ�������ʼ��
handles.samplet=0.001;       %����ʱ���ʼ��
handles.showtime=1000;       %��������(��ʾʱ��)��ʼ��
handles.e2sumall=[];         %���ƽ������ʷ��ʼ��
handles.kpall=[];            
handles.kiall=[];
handles.kdall=[];            %PID������ʷ��ʼ��
handles.sall=[];             %�ź�ѡ�������ʷ��ʼ��
handles.h=1;                 %��ʷ��������ʼ��
handles.kpmin=0;
handles.kimin=0;
handles.kdmin=0;             %���ƽ���͵��Լ�¼��СPID������ʼ��
handles.A=1;                 %λ��ʽPID��ֵ��ʼ��
handles.A1=0.8;
handles.A2=1.6;              %����ʽPID��ֵ��ʼ��

set(handles.edit7,'string',handles.samplet);
set(handles.edit8,'string',handles.A);
set(handles.edit9,'string',handles.showtime);       %�߼�ģʽ��ʾ��ʼ��

set(handles.uipanel6,'visible','off');
set(handles.uipanel4,'visible','off');
set(handles.uipanel7,'visible','off');
set(handles.pushbutton6,'visible','off');            %���ø߼�ģʽ�ɼ���

h=axes('units','normalized','position',[0 0 1 1]);   %���ñ���ͼƬ����
uistack(h,'bottom')                                  %���ñ���ͼ����λ�ã��ײ���
bgdata=imread('.\bg.jpg');            %��ȡ��Դ�������е�ͼƬ���浽bgdata
image(bgdata);                                       %��ͼƬ���ź������ʾ
colormap gray                                        %��ɫӳ��
set(h,'handlevisibility','off','visible','off');     %���������أ�ֻ��ʾͼ������

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes pidgui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = pidgui_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)        %���kp
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
handles.kp=str2double(get(hObject,'String'));
guidata(hObject, handles);

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



function edit2_Callback(hObject, eventdata, handles)      %���ki
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
handles.ki=str2double(get(hObject,'String'));
guidata(hObject, handles);


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



function edit3_Callback(hObject, eventdata, handles)        %���kd
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double
handles.kd=str2double(get(hObject,'String'));
guidata(hObject, handles);


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



function edit4_Callback(hObject, eventdata, handles)           %��÷���ϵ��
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double
handles.num=str2num(get(hObject,'String'));
guidata(hObject, handles);

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

function edit5_Callback(hObject, eventdata, handles)     %��÷�ĸϵ��
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double
handles.den=str2num(get(hObject,'String'));
guidata(hObject, handles);

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

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla(handles.axes1,'reset');      %����������1���ݣ��Ƴ������hold on�����ã�
kpl=handles.kp;
kil=handles.ki;
kdl=handles.kd;                  %�������䴫�ݱ���PID��������ú�����PID��������
numl=handles.num;
denl=handles.den;                %���������ӷ�ĸϵ����������ú����ķ��ӷ�ĸϵ���������
sys=tf(numl,denl);               %�������ض��󴫵ݺ���

%ʵ�ִ��ݺ��������
syms s;
gden=0;gnum=0;
for i=1:length(denl)
    gden=denl(i)*s^(length(denl)-i)+gden;
end
for i=1:length(numl)
    gnum=numl(i)*s^(length(numl)-i)+gnum;
end              
gden=char(gden);
gnum=char(gnum);
set(handles.text42,'string',gden);
set(handles.text41,'string',gnum);    

ts=handles.samplet;          %�趨����ʱ��
T=handles.showtime;
dsys=c2d(sys,ts,'z');        %�����ݺ�����ɢ��
[numl,denl]=tfdata(dsys,'v');%��ȡϵͳ���Ӻͷ�ĸȫ��ϵ��
eall=[];    %��һʱ��ƫ���ʼ��
esum=0;     %��ƫ���ʼ��
uall=[];    %��ʼ����һʱ��״̬����
yall=[];    %��ʼ����һʱ��״̬���
denorder=length(denl)-1; %�жϷ�ĸ����
numorder=length(numl)-1; %�жϷ��ӽ���
e2sum=0;                 %���ƽ���ͳ�ʼ��          
A=handles.A;             %���ܷ�ֵ��Ϣ
for i=1:denorder 
    uall(i)=0;
    yall(i)=0;
    eall(i)=0;           %��λ����һʱ���źų�ʼ��
end                     
for j=1:T                %��ʼ����
    time(j)=j*ts;        %����ʱ�����
    densum=0;            %�����ַ��̷�ĸ���ֳ�ʼ��
    numsum=0;            %�����ַ��̷�ĸ���ֳ�ʼ��
    r(j)=A;                            %Ĭ��״̬����������Ϊ��Ծ�ź�
    if handles.s==1                    %λ��ʽ�㷨���������ź�
        r(j)=A;                        %��������Ϊ��Ծ�ź�
    elseif handles.s==2
        r(j)=A*sign(sin(2*2*pi*j*ts)); %��������Ϊ�����ź�
    elseif handles.s==3
        r(j)=0.5*A*sin(2*2*pi*j*ts);   %��������Ϊ�����ź�
    end
    for k=1:denorder                   %�����ַ���
        densum=densum+((-1)*denl(k+1)*yall(k));
        numsum=numsum+numl(k+1)*uall(k);
    end
    y(j)=densum+numsum;                %�õ���Ӧ
    e(j)=r(j)-y(j);   %����ƫ��
    u(j)=kpl*e(j)+kil*esum+kdl*(e(j)-eall(1));
    esum=esum+e(j);   %��ƫ�������ƫ��
    if u(j)>=10       %����������޷�10
        u(j)=10;
    end
    if u(j)<=-10
        u(j)=-10;
    end
    for p=denorder:-1:2      %Ϊ��һʱ�̼�������ʼ��
        uall(p)=uall(p-1);
        yall(p)=yall(p-1);
        eall(p)=eall(p-1);
    end
    uall(1)=u(j);
    yall(1)=y(j);
    eall(1)=e(j);
    e2sum=e2sum+(e(j)).^2;   %�������ƽ����
end

%ϵͳ�ȶ�����˥����
dot=[];dotn1=0;dotn2=0;
dot=y;
[v,~]=findpeaks(dot);
dotn1=v(1)/v(2);
if handles.s==3
    set(handles.text44,'string','');
    set(handles.text45,'string','');
else
    set(handles.text44,'string',dotn1);
    if max(dotn1,v(2)/v(3))>1
        set(handles.text45,'string','�ȶ�');
    else
        set(handles.text45,'string','��ɢ');
    end
end

set(handles.text7,'string',esum);
set(handles.text10,'string',e(j));
set(handles.text17,'string',e2sum);       %������������ָ��
axes(handles.axes1)
plot(time,r,'r',time,y,'b','LineWidth',2);%�����Ӧͼ��
hold on;
plot(time,u,'g--');
grid on;
legend({'Expect curve','Output curve','Input curve'},'FontSize',8);
xlabel('time')
ylabel('output')

%���ø��������global����
global tz rz yz uz denz numz fres shtime fs;   
tz=time;
rz=r;yz=y;uz=u;
denz=handles.den;numz=handles.num;
fs=1/ts;
fres=(1/ts)*(0:T-1)/T;                  %Ƶ��ͼ��Ƶ������
shtime=T;                                 

%�����������
axes(handles.axes2)
plot(time,r,'r--',time,e,'b','LineWidth',2);
grid on;
xlabel('time')
ylabel('output')

%������ʷ��¼
handles.kpall(handles.h)=kpl;               
handles.kiall(handles.h)=kil;
handles.kdall(handles.h)=kdl;
handles.sall(handles.h)=handles.s;
handles.e2sumall(handles.h)=e2sum;
handles.h=handles.h+1;
set(handles.uitable1,'Data',[handles.e2sumall;handles.kpall;handles.kiall;handles.kdall;handles.sall]');

%��ȡ��ǰ����PID����
[e2summin,n]=min(handles.e2sumall);         
handles.kpmin=handles.kpall(n);
handles.kimin=handles.kiall(n);
handles.kdmin=handles.kdall(n);
set(handles.text22,'string',handles.kpmin);
set(handles.text23,'string',handles.kimin);
set(handles.text24,'string',handles.kdmin);
guidata(hObject, handles);

% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
S=get(handles.popupmenu1,'value');   
switch S
    case 1
        handles.s=1;                 
    case 2
        handles.s=2;
    case 3
        handles.s=3;
%     case 4
%         handles.s=4
%     case 5
%         handles.s=5
end                            %����ź�ѡ����������뺯���䴫�ݱ���handles.s
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global tz rz yz uz denz numz fres shtime fs;
fn=255;
open('perfAnaly.fig');
p=guihandles;
G=tf(numz,denz);
y=fft(yz',shtime);        %��ʱ����Ҷ�任
mag=abs(y);               %�ö�Ӧ��ֵ

axes(p.axes1)             %����Ƶ��ͼ
plot(fres(1:shtime/2),mag(1:shtime/2),'r','LineWidth',1.5)
set(gca,'XLim',[0 0.1*fs]);%����x����ʾ��Χ

axes(p.axes2)             %�����ο�˹������
nyquist(G)

axes(p.axes3)

% t=tz;
% a=yz;
% T=length(a);
% [tfr,t,f] =tfrstft(a',1:T,T,hamming(501),0);
% %contour(t/fs,f(1:length(f)/2)*fs,abs(tfr(1:length(f)/2,:)));
% mesh(t/fs,f(1:length(f)/2)*fs,abs(tfr(1:length(f)/2,:)));

if fs<256
    fn=fs;
else
    fn=255;
end              %noverlapӰ��ʱ����ķֱ��ʣ�Խ�ӽ�nfft���ֱ���Խ��
[y,f,t,k] = spectrogram(yz(1:shtime/2),256,fn,fres(1:shtime/2),fs,'yaxis'); %freqloc�ַ������Կ���Ƶ������ʾ��λ��
surf(t,f,10*log10(abs(k)),'EdgeColor','none');   %��ͼ
axis ij;    %�趨ȡֵ����
axis tight; %�����귶Χ�趨Ϊ�����Ƶ����ݷ�Χ
xlabel('ʱ��');
ylabel('Ƶ��(Hz)');

axes(p.axes4)         %����Bodeͼ
margin(G)

% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close;          %�ر�GUI

% --------------------------------------------------------------------
function Untitled_2_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_3_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_4_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.axes1,'reset');
cla(handles.axes2,'reset');
set(handles.text7,'string','');
set(handles.text10,'string','');
set(handles.text17,'string','');          %����ͼ��

% --------------------------------------------------------------------
function Untitled_5_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.edit1,'string','');
set(handles.edit2,'string','');
set(handles.edit3,'string','');
set(handles.text7,'string','');
set(handles.text10,'string','');
set(handles.text17,'string','');
set(handles.text44,'string','');
set(handles.text45,'string','');%����PID����

% --------------------------------------------------------------------
function Untitled_6_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.edit4,'string','');
set(handles.edit5,'string','');
set(handles.text7,'string','');
set(handles.text10,'string','');
set(handles.text17,'string','');
set(handles.text41,'string','');
set(handles.text42,'string','');
set(handles.text44,'string','');
set(handles.text45,'string','');%���ô��ݺ�������
 

% --------------------------------------------------------------------
function Untitled_7_Callback(hObject, eventdata, handles)      
% hObject    handle to Untitled_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%��ʼ������ȫ������
set(handles.edit1,'string','');
set(handles.edit2,'string','');
set(handles.edit3,'string','');
set(handles.edit4,'string','');
set(handles.edit5,'string','');
set(handles.text7,'string','');
set(handles.text10,'string','');
set(handles.text17,'string','');
set(handles.text44,'string','');
set(handles.text45,'string','');
set(handles.text41,'string','');
set(handles.text42,'string','');
set(handles.uitable1,'Data','');
set(handles.text22,'string','');
set(handles.text23,'string','');
set(handles.text24,'string','');
cla(handles.axes1,'reset');
cla(handles.axes2,'reset');

handles.kp=0;
handles.ki=0;
handles.kd=0;
handles.num=[];
handles.den=[];
handles.s=0;
handles.samplet=0.001;        
handles.showtime=1000;       
handles.e2sumall=[];
handles.kpall=[];
handles.kiall=[];
handles.kdall=[];
handles.sall=[];
handles.h=1;                  
handles.kpmin=0;
handles.kimin=0;
handles.kdmin=0;
handles.A=1;
handles.A1=0.8;
handles.A2=1.6;
set(handles.edit7,'string',handles.samplet);
set(handles.edit9,'string',handles.A);
set(handles.edit8,'string',handles.showtime);
guidata(hObject, handles);

% --------------------------------------------------------------------
function Untitled_8_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_9_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_10_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_11_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_16_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Ԥ��PID��������Ծ
global KpSaveS1;
global KiSaveS1;
global KdSaveS1;
if KpSaveS1      %�ȴӱ����Ԥ���ж�ȡ
    set(handles.edit1,'string',KpSaveS1);
    set(handles.edit2,'string',KiSaveS1);
    set(handles.edit3,'string',KdSaveS1);
    handles.kp=KpSaveS1;
    handles.ki=KiSaveS1;
    handles.kd=KdSaveS1;
else             %û�еĻ�ʹ��Ĭ��Ԥ��
    set(handles.edit1,'string','1.09');
    set(handles.edit2,'string','0.000235');
    set(handles.edit3,'string','9.8');
    handles.kp=1.09;
    handles.ki=0.000235;
    handles.kd=9.8;
end 
guidata(hObject, handles);

% --------------------------------------------------------------------
function Untitled_17_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Ԥ��PID����������
global KpSaveS2;
global KiSaveS2;
global KdSaveS2;
if KpSaveS2
    set(handles.edit1,'string',KpSaveS2);
    set(handles.edit2,'string',KiSaveS2);
    set(handles.edit3,'string',KdSaveS2);
    handles.kp=KpSaveS2;
    handles.ki=KiSaveS2;
    handles.kd=KdSaveS2;
else
    set(handles.edit1,'string','1.2');
    set(handles.edit2,'string','0');
    set(handles.edit3,'string','7.5');
    handles.kp=1.2;
    handles.ki=0;
    handles.kd=7.5;
end
guidata(hObject, handles);

% --------------------------------------------------------------------
function Untitled_18_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Ԥ��PID����������
global KpSaveS3;
global KiSaveS3;
global KdSaveS3;
if KpSaveS3
    set(handles.edit1,'string',KpSaveS3);
    set(handles.edit2,'string',KiSaveS3);
    set(handles.edit3,'string',KdSaveS3);
    handles.kp=KpSaveS3;
    handles.ki=KiSaveS3;
    handles.kd=KdSaveS3;
else
    set(handles.edit1,'string','1.7');
    set(handles.edit2,'string','0');
    set(handles.edit3,'string','150');
    handles.kp=1.7;
    handles.ki=0;
    handles.kd=150;
end
guidata(hObject, handles);

% --------------------------------------------------------------------
function Untitled_15_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Ԥ�贫����������Ŀ
set(handles.edit4,'string','523500');
set(handles.edit5,'string','1 87.35 10470 0');
handles.num=[523500];
handles.den=[1 87.35 10470 0];
guidata(hObject, handles);


% --------------------------------------------------------------------
function Untitled_19_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Ԥ��PID����������
set(handles.edit1,'string','0.7');
set(handles.edit2,'string','0.001');
set(handles.edit3,'string','1.5');
handles.kp=0.7;
handles.ki=0.001;
handles.kd=1.5;
guidata(hObject, handles);


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_20_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%[file,path,index]=uiputfile({'*.png';'*.bmp';'*.jpg'},'Save As');

%���浱ǰ������1��ͼƬ
new_handle=figure('visible','off');                       
new_axes=copyobj(handles.axes1,new_handle);
set(new_axes,'units','default','position','default');
[file,path,index]=uiputfile({'*.jpg';'*.bmp'},'save as');
if ~file                           %�ж������ļ�
    return
else
    file=strcat(path,file);
    switch index                   %���ݲ�ͬ��ѡ�񱣴�Ϊ��ͬ������
        case 1
            print(new_handle,'-djpeg',file);
        case 2
            print(new_handle,'-dbmp',file);
    end
end
delete(new_handle);


% --------------------------------------------------------------------
function Untitled_21_Callback(hObject, eventdata, handles)    %��ʷ��¼����
% hObject    handle to Untitled_21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.uitable1,'Data','');
set(handles.text22,'string','');
set(handles.text23,'string','');
set(handles.text24,'string','');
handles.e2sumall=[];
handles.kpall=[];
handles.kiall=[];
handles.kdall=[];
handles.sall=[];
handles.h=1;                          %������ʷ��¼
guidata(hObject, handles);



% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
get(hObject,'Value')

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

%���÷�ֵ
handles.A=get(hObject,'Value');
set(handles.edit8,'string',handles.A);      
guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

%������ʾʱ��
handles.showtime=get(hObject,'Value');
set(handles.edit9,'string',handles.showtime);
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

%���ò���ʱ��
handles.samplet=get(hObject,'Value');
set(handles.edit7,'string',handles.samplet);
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes when selected object is changed in uipanel5.
function uipanel5_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uipanel5
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)

%�߼�ģʽ/���ģʽ�л�
switch get(eventdata.NewValue,'Tag')
    case 'radiobutton2'
        set(handles.uipanel6,'visible','off');
        set(handles.uipanel4,'visible','off');
        set(handles.uipanel7,'visible','off');
        set(handles.pushbutton6,'visible','off');
    case 'radiobutton1'
        set(handles.uipanel6,'visible','on');
        set(handles.uipanel4,'visible','on');
        set(handles.uipanel7,'visible','on');
        set(handles.pushbutton6,'visible','on');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)%����ʽ����
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla(handles.axes1,'reset');      %����������1���ݣ��Ƴ������hold on�����ã�
kpl=handles.kp;
kil=handles.ki;
kdl=handles.kd;                  %�������䴫�ݱ���PID��������ú�����PID��������
numl=handles.num;
denl=handles.den;                %���������ӷ�ĸϵ����������ú����ķ��ӷ�ĸϵ���������
sys=tf(numl,denl);               %�������ض��󴫵ݺ���

%ʵ�ִ��ݺ��������
syms s;
gden=0;gnum=0;
for i=1:length(denl)
    gden=denl(i)*s^(length(denl)-i)+gden;
end
for i=1:length(numl)
    gnum=numl(i)*s^(length(numl)-i)+gnum;
end
gden=char(gden);
gnum=char(gnum);
set(handles.text42,'string',gden);
set(handles.text41,'string',gnum);    %ʵ�ִ��ݺ��������



ts=handles.samplet;          %�趨����ʱ��
T=handles.showtime;
dsys=c2d(sys,ts,'z');        %�����ݺ�����ɢ��
[numl,denl]=tfdata(dsys,'v');%��ȡϵͳ���Ӻͷ�ĸȫ��ϵ��
eall=[];    %��һʱ��ƫ���ʼ��
esum=0;     %��ƫ���ʼ��
uall=[];    %��ʼ����һʱ��״̬����
yall=[];    %��ʼ����һʱ��״̬���
denorder=length(denl)-1; %�жϷ�ĸ����
numorder=length(numl)-1; %�жϷ��ӽ���
e2sum=0;                 %���ƽ���ͳ�ʼ��   
A1=handles.A1;
A2=handles.A2;
for i=1:denorder
    uall(i)=0;
    yall(i)=0;
    eall(i)=0;
end
for j=1:T      %��ʼ����
    time(j)=j*ts;     %����ʱ�����
    densum=0;
    numsum=0;
    if j<0.5*T
        r(j)=A1;
    else
        r(j)=A2;
    end
    if j<0.5*T
        if handles.s==1   %λ��ʽ�㷨���������ź�
            r(j)=A1;       %��������Ϊ��Ծ�ź�
        elseif handles.s==2
            r(j)=A1*sign(sin(2*2*pi*j*ts)); %��������Ϊ�����ź�
        elseif handles.s==3
            r(j)=0.5*A1*sin(2*2*pi*j*ts);   %��������Ϊ�����ź�
        end
    else
        if handles.s==1   %λ��ʽ�㷨���������ź�
            r(j)=A2;       %��������Ϊ��Ծ�ź�
        elseif handles.s==2
            r(j)=A2*sign(sin(2*2*pi*j*ts)); %��������Ϊ�����ź�
        elseif handles.s==3
            r(j)=0.5*A2*sin(2*2*pi*j*ts);   %��������Ϊ�����ź�
        end
    end
    for k=1:denorder                 %�����ַ���
        densum=densum+((-1)*denl(k+1)*yall(k));
        numsum=numsum+numl(k+1)*uall(k);
    end
    y(j)=densum+numsum;
    e(j)=r(j)-y(j);   %����ƫ��
    u(j)=kpl*e(j)+kil*esum+kdl*(e(j)-eall(1));
    esum=esum+e(j);   %��ƫ�������ƫ��
    if u(j)>=10       %�޷�10
        u(j)=10;
    end
    if u(j)<=-10
        u(j)=-10;
    end
    for p=denorder:-1:2      %Ϊ��һʱ�̼�������ʼ��
        uall(p)=uall(p-1);
        yall(p)=yall(p-1);
        eall(p)=eall(p-1);
    end
    uall(1)=u(j);
    yall(1)=y(j);
    eall(1)=e(j);
    e2sum=e2sum+(e(j)).^2; %���ƽ����
end

dot=[];dotn1=0;dotn2=0;
dot=y;
[v,~]=findpeaks(dot);
dotn1=v(1)/v(2);
if handles.s==3
    set(handles.text44,'string','');
    set(handles.text45,'string','');
else
    set(handles.text44,'string',dotn1);
    if max(dotn1,v(2)/v(3))>1
        set(handles.text45,'string','�ȶ�');
    else
        set(handles.text45,'string','��ɢ');
    end
end

set(handles.text7,'string',esum);
set(handles.text10,'string',e(j));
set(handles.text17,'string',e2sum);       %������������ָ��
axes(handles.axes1)
plot(time,r,'r',time,y,'b','LineWidth',2);%�����Ӧͼ��
hold on;
plot(time,u,'g--');
grid on;
legend({'Expect curve','Output curve','Input curve'},'FontSize',8);
xlabel('time')
ylabel('output')


%���ø��������global����
global tz rz yz uz denz numz fres shtime fs;   
tz=time;
rz=r;yz=y;uz=u;
denz=handles.den;numz=handles.num;
fs=1/ts;
fres=(1/ts)*(0:T-1)/T;                  %Ƶ��ͼ��Ƶ������
shtime=T;                                 

%�����������
axes(handles.axes2)
plot(time,r,'r--',time,e,'b','LineWidth',2);
grid on;
xlabel('time')
ylabel('output')

%������ʷ��¼
handles.kpall(handles.h)=kpl;               
handles.kiall(handles.h)=kil;
handles.kdall(handles.h)=kdl;
handles.sall(handles.h)=handles.s;
handles.e2sumall(handles.h)=e2sum;
handles.h=handles.h+1;
set(handles.uitable1,'Data',[handles.e2sumall;handles.kpall;handles.kiall;handles.kdall;handles.sall]');

%��ȡ��ǰ����PID����
[e2summin,n]=min(handles.e2sumall);         
handles.kpmin=handles.kpall(n);
handles.kimin=handles.kiall(n);
handles.kdmin=handles.kdall(n);
set(handles.text22,'string',handles.kpmin);
set(handles.text23,'string',handles.kimin);
set(handles.text24,'string',handles.kdmin);
guidata(hObject, handles);

% --------------------------------------------------------------------
function Untitled_22_Callback(hObject, eventdata, handles)%һ������
% hObject    handle to Untitled_22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.edit4,'string','523500');
set(handles.edit5,'string','1 87.35 10470 0');
handles.num=[523500];
handles.den=[1 87.35 10470 0];
set(handles.edit1,'string','0.7');
set(handles.edit2,'string','0.001');
set(handles.edit3,'string','1.5');
handles.kp=0.7;
handles.ki=0.001;
handles.kd=1.5;
guidata(hObject, handles);


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%����Ԥ��
if handles.s==1
    global KpSaveS1;
    global KiSaveS1;
    global KdSaveS1;
    KpSaveS1=handles.kpmin;
    KiSaveS1=handles.kimin;
    KdSaveS1=handles.kdmin;
elseif handles.s==2
    global KpSaveS2;
    global KiSaveS2;
    global KdSaveS2;
    KpSaveS2=handles.kpmin;
    KiSaveS2=handles.kimin;
    KdSaveS2=handles.kdmin;
elseif handles.s==3
    global KpSaveS3;
    global KiSaveS3;
    global KdSaveS3;
    KpSaveS3=handles.kpmin;
    KiSaveS3=handles.kimin;
    KdSaveS3=handles.kdmin;
end
guidata(hObject, handles);


% --------------------------------------------------------------------
function Untitled_24_Callback(hObject, eventdata, handles)%ͼƬ�Ŵ�
% hObject    handle to Untitled_24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guide(figaxes);

% global tz rz yz uz;
% open('figaxes.fig');
% h=guihandles;
% axes(h.axes1)
% plot(tz,rz,'r',tz,yz,'b',tz,uz,'g');




% --------------------------------------------------------------------
function ZOOM_Callback(hObject, eventdata, handles)
% hObject    handle to ZOOM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_25_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%���浱ǰ������1��ͼƬ
new_handle=figure('visible','off');                       
new_axes=copyobj(handles.axes1,new_handle);
set(new_axes,'units','default','position','default');
[file,path,index]=uiputfile({'*.jpg';'*.bmp'},'save as');
if ~file
    return
else
    file=strcat(path,file);
    switch index %���ݲ�ͬ��ѡ�񱣴�Ϊ��ͬ������
        case 1
            print(new_handle,'-djpeg',file);
        case 2
            print(new_handle,'-dbmp',file);
    end
end
delete(new_handle);

% --------------------------------------------------------------------
function Untitled_28_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.axes1,'reset');
set(handles.text7,'string','');
set(handles.text10,'string','');
set(handles.text17,'string','');       %�Ҽ����ͼƬ


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)%����
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
open('guihelp.fig');

% --------------------------------------------------------------------
function Untitled_29_Callback(hObject, eventdata, handles)%����
% hObject    handle to Untitled_29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
open('guihelp.fig');



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double
handles.samplet=str2num(get(hObject,'String'));
set(handles.slider4,'value',handles.samplet);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double
handles.A=str2num(get(hObject,'String'));
set(handles.slider2,'value',handles.A);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double
handles.showtime=str2num(get(hObject,'String'));
set(handles.slider3,'value',handles.showtime);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function Untitled_31_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_33_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function ZOOM2_Callback(hObject, eventdata, handles)
% hObject    handle to ZOOM2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
