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

% I=imread('H:\matlab_design\logo.jpg');     %写入logo图片的数据
% javaImage=im2java(I);                      %转换成java语言
% newIcon=javax.swing.ImageIcon(javaImage);  %
% figFrame=get(handles.figure1,'JavaFrame'); %取得Figure的JavaFrame。
% figFrame.setFigureIcon(newIcon);           %修改图标

%函数间传递变量初始化
handles.kp=0;                
handles.ki=0;
handles.kd=0;                %PID参数初始化
handles.num=[];
handles.den=[];              %传递函数分子分母系数初始化
handles.s=0;                 %信号选择变量初始化
handles.samplet=0.001;       %采样时间初始化
handles.showtime=1000;       %采样次数(显示时间)初始化
handles.e2sumall=[];         %误差平方和历史初始化
handles.kpall=[];            
handles.kiall=[];
handles.kdall=[];            %PID参数历史初始化
handles.sall=[];             %信号选择变量历史初始化
handles.h=1;                 %历史计数器初始化
handles.kpmin=0;
handles.kimin=0;
handles.kdmin=0;             %误差平方和调试记录最小PID参数初始化
handles.A=1;                 %位置式PID幅值初始化
handles.A1=0.8;
handles.A2=1.6;              %增量式PID幅值初始化

set(handles.edit7,'string',handles.samplet);
set(handles.edit8,'string',handles.A);
set(handles.edit9,'string',handles.showtime);       %高级模式显示初始化

set(handles.uipanel6,'visible','off');
set(handles.uipanel4,'visible','off');
set(handles.uipanel7,'visible','off');
set(handles.pushbutton6,'visible','off');            %设置高级模式可见性

h=axes('units','normalized','position',[0 0 1 1]);   %设置背景图片画布
uistack(h,'bottom')                                  %设置背景图叠放位置（底部）
bgdata=imread('.\bg.jpg');            %读取资源管理器中的图片保存到bgdata
image(bgdata);                                       %将图片缩放后进行显示
colormap gray                                        %颜色映射
set(h,'handlevisibility','off','visible','off');     %将画布隐藏，只显示图像内容

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



function edit1_Callback(hObject, eventdata, handles)        %获得kp
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



function edit2_Callback(hObject, eventdata, handles)      %获得ki
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



function edit3_Callback(hObject, eventdata, handles)        %获得kd
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



function edit4_Callback(hObject, eventdata, handles)           %获得分子系数
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

function edit5_Callback(hObject, eventdata, handles)     %获得分母系数
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

cla(handles.axes1,'reset');      %重置坐标轴1内容（移除后面的hold on的作用）
kpl=handles.kp;
kil=handles.ki;
kdl=handles.kd;                  %将函数间传递变量PID参数赋予该函数的PID参数变量
numl=handles.num;
denl=handles.den;                %将传函分子分母系数参数赋予该函数的分子分母系数数组变量
sys=tf(numl,denl);               %建立被控对象传递函数

%实现传递函数的输出
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

ts=handles.samplet;          %设定采样时间
T=handles.showtime;
dsys=c2d(sys,ts,'z');        %将传递函数离散化
[numl,denl]=tfdata(dsys,'v');%获取系统分子和分母全部系数
eall=[];    %上一时刻偏差初始化
esum=0;     %总偏差初始化
uall=[];    %初始化上一时刻状态输入
yall=[];    %初始化上一时刻状态输出
denorder=length(denl)-1; %判断分母阶数
numorder=length(numl)-1; %判断分子阶数
e2sum=0;                 %误差平方和初始化          
A=handles.A;             %接受幅值信息
for i=1:denorder 
    uall(i)=0;
    yall(i)=0;
    eall(i)=0;           %各位置上一时刻信号初始化
end                     
for j=1:T                %开始采样
    time(j)=j*ts;        %计算时间参数
    densum=0;            %输出差分方程分母部分初始化
    numsum=0;            %输出差分方程分母部分初始化
    r(j)=A;                            %默认状态下输入设置为阶跃信号
    if handles.s==1                    %位置式算法设置输入信号
        r(j)=A;                        %输入设置为阶跃信号
    elseif handles.s==2
        r(j)=A*sign(sin(2*2*pi*j*ts)); %输入设置为方波信号
    elseif handles.s==3
        r(j)=0.5*A*sin(2*2*pi*j*ts);   %输入设置为正弦信号
    end
    for k=1:denorder                   %计算差分方程
        densum=densum+((-1)*denl(k+1)*yall(k));
        numsum=numsum+numl(k+1)*uall(k);
    end
    y(j)=densum+numsum;                %得到响应
    e(j)=r(j)-y(j);   %计算偏差
    u(j)=kpl*e(j)+kil*esum+kdl*(e(j)-eall(1));
    esum=esum+e(j);   %将偏差计入总偏差
    if u(j)>=10       %控制器输出限幅10
        u(j)=10;
    end
    if u(j)<=-10
        u(j)=-10;
    end
    for p=denorder:-1:2      %为下一时刻计算做初始化
        uall(p)=uall(p-1);
        yall(p)=yall(p-1);
        eall(p)=eall(p-1);
    end
    uall(1)=u(j);
    yall(1)=y(j);
    eall(1)=e(j);
    e2sum=e2sum+(e(j)).^2;   %计算误差平方和
end

%系统稳定性与衰减比
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
        set(handles.text45,'string','稳定');
    else
        set(handles.text45,'string','发散');
    end
end

set(handles.text7,'string',esum);
set(handles.text10,'string',e(j));
set(handles.text17,'string',e2sum);       %设置性能评估指标
axes(handles.axes1)
plot(time,r,'r',time,y,'b','LineWidth',2);%输出响应图像
hold on;
plot(time,u,'g--');
grid on;
legend({'Expect curve','Output curve','Input curve'},'FontSize',8);
xlabel('time')
ylabel('output')

%设置更多分析的global变量
global tz rz yz uz denz numz fres shtime fs;   
tz=time;
rz=r;yz=y;uz=u;
denz=handles.den;numz=handles.num;
fs=1/ts;
fres=(1/ts)*(0:T-1)/T;                  %频域图的频率序列
shtime=T;                                 

%绘制误差曲线
axes(handles.axes2)
plot(time,r,'r--',time,e,'b','LineWidth',2);
grid on;
xlabel('time')
ylabel('output')

%设置历史记录
handles.kpall(handles.h)=kpl;               
handles.kiall(handles.h)=kil;
handles.kdall(handles.h)=kdl;
handles.sall(handles.h)=handles.s;
handles.e2sumall(handles.h)=e2sum;
handles.h=handles.h+1;
set(handles.uitable1,'Data',[handles.e2sumall;handles.kpall;handles.kiall;handles.kdall;handles.sall]');

%获取当前最优PID参数
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
end                            %获得信号选择变量并存入函数间传递变量handles.s
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
y=fft(yz',shtime);        %短时傅里叶变换
mag=abs(y);               %得对应幅值

axes(p.axes1)             %绘制频域图
plot(fres(1:shtime/2),mag(1:shtime/2),'r','LineWidth',1.5)
set(gca,'XLim',[0 0.1*fs]);%设置x轴显示范围

axes(p.axes2)             %绘制奈奎斯特曲线
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
end              %noverlap影响时间轴的分辨率，越接近nfft，分辨率越高
[y,f,t,k] = spectrogram(yz(1:shtime/2),256,fn,fres(1:shtime/2),fs,'yaxis'); %freqloc字符串可以控制频率轴显示的位置
surf(t,f,10*log10(abs(k)),'EdgeColor','none');   %绘图
axis ij;    %设定取值方向
axis tight; %将坐标范围设定为被绘制的数据范围
xlabel('时间');
ylabel('频率(Hz)');

axes(p.axes4)         %绘制Bode图
margin(G)

% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close;          %关闭GUI

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
set(handles.text17,'string','');          %重置图像

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
set(handles.text45,'string','');%重置PID参数

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
set(handles.text45,'string','');%重置传递函数数据
 

% --------------------------------------------------------------------
function Untitled_7_Callback(hObject, eventdata, handles)      
% hObject    handle to Untitled_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%初始化界面全部内容
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

%预设PID参数：阶跃
global KpSaveS1;
global KiSaveS1;
global KdSaveS1;
if KpSaveS1      %先从保存的预设中读取
    set(handles.edit1,'string',KpSaveS1);
    set(handles.edit2,'string',KiSaveS1);
    set(handles.edit3,'string',KdSaveS1);
    handles.kp=KpSaveS1;
    handles.ki=KiSaveS1;
    handles.kd=KdSaveS1;
else             %没有的话使用默认预设
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

%预设PID参数：方波
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

%预设PID参数：正弦
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

%预设传函：课设题目
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

%预设PID参数：测试
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

%保存当前坐标轴1的图片
new_handle=figure('visible','off');                       
new_axes=copyobj(handles.axes1,new_handle);
set(new_axes,'units','default','position','default');
[file,path,index]=uiputfile({'*.jpg';'*.bmp'},'save as');
if ~file                           %判断有无文件
    return
else
    file=strcat(path,file);
    switch index                   %根据不同的选择保存为不同的类型
        case 1
            print(new_handle,'-djpeg',file);
        case 2
            print(new_handle,'-dbmp',file);
    end
end
delete(new_handle);


% --------------------------------------------------------------------
function Untitled_21_Callback(hObject, eventdata, handles)    %历史记录重置
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
handles.h=1;                          %重置历史记录
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

%设置幅值
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

%设置显示时间
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

%设置采样时间
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

%高级模式/简洁模式切换
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
function pushbutton6_Callback(hObject, eventdata, handles)%增量式跟踪
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla(handles.axes1,'reset');      %重置坐标轴1内容（移除后面的hold on的作用）
kpl=handles.kp;
kil=handles.ki;
kdl=handles.kd;                  %将函数间传递变量PID参数赋予该函数的PID参数变量
numl=handles.num;
denl=handles.den;                %将传函分子分母系数参数赋予该函数的分子分母系数数组变量
sys=tf(numl,denl);               %建立被控对象传递函数

%实现传递函数的输出
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
set(handles.text41,'string',gnum);    %实现传递函数的输出



ts=handles.samplet;          %设定采样时间
T=handles.showtime;
dsys=c2d(sys,ts,'z');        %将传递函数离散化
[numl,denl]=tfdata(dsys,'v');%获取系统分子和分母全部系数
eall=[];    %上一时刻偏差初始化
esum=0;     %总偏差初始化
uall=[];    %初始化上一时刻状态输入
yall=[];    %初始化上一时刻状态输出
denorder=length(denl)-1; %判断分母阶数
numorder=length(numl)-1; %判断分子阶数
e2sum=0;                 %误差平方和初始化   
A1=handles.A1;
A2=handles.A2;
for i=1:denorder
    uall(i)=0;
    yall(i)=0;
    eall(i)=0;
end
for j=1:T      %开始采样
    time(j)=j*ts;     %计算时间参数
    densum=0;
    numsum=0;
    if j<0.5*T
        r(j)=A1;
    else
        r(j)=A2;
    end
    if j<0.5*T
        if handles.s==1   %位置式算法设置输入信号
            r(j)=A1;       %输入设置为阶跃信号
        elseif handles.s==2
            r(j)=A1*sign(sin(2*2*pi*j*ts)); %输入设置为方波信号
        elseif handles.s==3
            r(j)=0.5*A1*sin(2*2*pi*j*ts);   %输入设置为正弦信号
        end
    else
        if handles.s==1   %位置式算法设置输入信号
            r(j)=A2;       %输入设置为阶跃信号
        elseif handles.s==2
            r(j)=A2*sign(sin(2*2*pi*j*ts)); %输入设置为方波信号
        elseif handles.s==3
            r(j)=0.5*A2*sin(2*2*pi*j*ts);   %输入设置为正弦信号
        end
    end
    for k=1:denorder                 %计算差分方程
        densum=densum+((-1)*denl(k+1)*yall(k));
        numsum=numsum+numl(k+1)*uall(k);
    end
    y(j)=densum+numsum;
    e(j)=r(j)-y(j);   %计算偏差
    u(j)=kpl*e(j)+kil*esum+kdl*(e(j)-eall(1));
    esum=esum+e(j);   %将偏差计入总偏差
    if u(j)>=10       %限幅10
        u(j)=10;
    end
    if u(j)<=-10
        u(j)=-10;
    end
    for p=denorder:-1:2      %为下一时刻计算做初始化
        uall(p)=uall(p-1);
        yall(p)=yall(p-1);
        eall(p)=eall(p-1);
    end
    uall(1)=u(j);
    yall(1)=y(j);
    eall(1)=e(j);
    e2sum=e2sum+(e(j)).^2; %误差平方和
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
        set(handles.text45,'string','稳定');
    else
        set(handles.text45,'string','发散');
    end
end

set(handles.text7,'string',esum);
set(handles.text10,'string',e(j));
set(handles.text17,'string',e2sum);       %设置性能评估指标
axes(handles.axes1)
plot(time,r,'r',time,y,'b','LineWidth',2);%输出响应图像
hold on;
plot(time,u,'g--');
grid on;
legend({'Expect curve','Output curve','Input curve'},'FontSize',8);
xlabel('time')
ylabel('output')


%设置更多分析的global变量
global tz rz yz uz denz numz fres shtime fs;   
tz=time;
rz=r;yz=y;uz=u;
denz=handles.den;numz=handles.num;
fs=1/ts;
fres=(1/ts)*(0:T-1)/T;                  %频域图的频率序列
shtime=T;                                 

%绘制误差曲线
axes(handles.axes2)
plot(time,r,'r--',time,e,'b','LineWidth',2);
grid on;
xlabel('time')
ylabel('output')

%设置历史记录
handles.kpall(handles.h)=kpl;               
handles.kiall(handles.h)=kil;
handles.kdall(handles.h)=kdl;
handles.sall(handles.h)=handles.s;
handles.e2sumall(handles.h)=e2sum;
handles.h=handles.h+1;
set(handles.uitable1,'Data',[handles.e2sumall;handles.kpall;handles.kiall;handles.kdall;handles.sall]');

%获取当前最优PID参数
[e2summin,n]=min(handles.e2sumall);         
handles.kpmin=handles.kpall(n);
handles.kimin=handles.kiall(n);
handles.kdmin=handles.kdall(n);
set(handles.text22,'string',handles.kpmin);
set(handles.text23,'string',handles.kimin);
set(handles.text24,'string',handles.kdmin);
guidata(hObject, handles);

% --------------------------------------------------------------------
function Untitled_22_Callback(hObject, eventdata, handles)%一键测试
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

%保存预设
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
function Untitled_24_Callback(hObject, eventdata, handles)%图片放大
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

%保存当前坐标轴1的图片
new_handle=figure('visible','off');                       
new_axes=copyobj(handles.axes1,new_handle);
set(new_axes,'units','default','position','default');
[file,path,index]=uiputfile({'*.jpg';'*.bmp'},'save as');
if ~file
    return
else
    file=strcat(path,file);
    switch index %根据不同的选择保存为不同的类型
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
set(handles.text17,'string','');       %右键清除图片


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)%帮助
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
open('guihelp.fig');

% --------------------------------------------------------------------
function Untitled_29_Callback(hObject, eventdata, handles)%帮助
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
