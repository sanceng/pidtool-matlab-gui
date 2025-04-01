global yz fres shtime fs;
fn=255;
if fs<256
    fn=fs;
else
    fn=255;
end    %noverlap影响时间轴的分辨率，越接近nfft，分辨率越高
[y,f,t,k] = spectrogram(yz(1:shtime/2),256,fn,fres(1:shtime/2),fs,'yaxis'); %freqloc字符串可以控制频率轴显示的位置
surf(t,f,10*log10(abs(k)),'EdgeColor','none');   %绘图
axis ij;    %设定取值方向
axis tight; %将坐标范围设定为被绘制的数据范围
xlabel('时间');
ylabel('频率(Hz)');