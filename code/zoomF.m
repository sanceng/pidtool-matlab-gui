global yz fres shtime fs;
fn=255;
if fs<256
    fn=fs;
else
    fn=255;
end    %noverlapӰ��ʱ����ķֱ��ʣ�Խ�ӽ�nfft���ֱ���Խ��
[y,f,t,k] = spectrogram(yz(1:shtime/2),256,fn,fres(1:shtime/2),fs,'yaxis'); %freqloc�ַ������Կ���Ƶ������ʾ��λ��
surf(t,f,10*log10(abs(k)),'EdgeColor','none');   %��ͼ
axis ij;    %�趨ȡֵ����
axis tight; %�����귶Χ�趨Ϊ�����Ƶ����ݷ�Χ
xlabel('ʱ��');
ylabel('Ƶ��(Hz)');