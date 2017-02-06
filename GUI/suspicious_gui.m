function varargout = suspicious_gui(varargin)
% SUSPICIOUS_GUI MATLAB code for suspicious_gui.fig
%      SUSPICIOUS_GUI, by itself, creates a new SUSPICIOUS_GUI or raises the existing
%      singleton*.
%
%      H = SUSPICIOUS_GUI returns the handle to a new SUSPICIOUS_GUI or the handle to
%      the existing singleton*.
%
%      SUSPICIOUS_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SUSPICIOUS_GUI.M with the given input arguments.
%
%      SUSPICIOUS_GUI('Property','Value',...) creates a new SUSPICIOUS_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before suspicious_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to suspicious_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help suspicious_gui

% Last Modified by GUIDE v2.5 12-May-2016 16:47:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @suspicious_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @suspicious_gui_OutputFcn, ...
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


% --- Executes just before suspicious_gui is made visible.
function suspicious_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to suspicious_gui (see VARARGIN)

% Choose default command line output for suspicious_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes suspicious_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = suspicious_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in Background_Substraction.
function Background_Substraction_Callback(hObject, eventdata, handles)
% hObject    handle to Background_Substraction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Background=imread('background.jpg');
%CurrentFrame=imread('current.jpg');
Background=handles.b;
CurrentFrame=handles.c;
%Convert RGB 2 HSV Color conversion
[Background_hsv]=round(rgb2hsv(Background));
[CurrentFrame_hsv]=round(rgb2hsv(CurrentFrame));
Out = bitxor(Background_hsv,CurrentFrame_hsv);
%Convert RGB 2 GRAY
%Out=rgb2gray(Out);
%Read Rows and Columns of the Image
[rows columns]=size(Out);
%Convert to Binary Image
for i=1:rows
for j=1:columns

if Out(i,j) >0

BinaryImage(i,j)=1;

else

BinaryImage(i,j)=0;

end

end
end

%Apply Median filter to remove Noise
FilteredImage=medfilt2(BinaryImage,[5 5]);

%Boundary Label the Filtered Image
[L num]=bwlabel(FilteredImage);

STATS=regionprops(L,'all');
cc=[];
removed=0;

%Remove the noisy regions
for i=1:num
dd=STATS(i).Area;

if (dd < 500)

L(L==i)=0;
removed = removed + 1;
num=num-1;

else

end

end

[L2 num2]=bwlabel(L);

% Trace region boundaries in a binary image.

[B,L,N,A] = bwboundaries(L2);

%Display results
 
   axes(handles.axes3);
   imshow(L2); 
    guidata(hObject,handles);

hold on;

%for k=1:length(B),

%if(~sum(A(k,:)))
%boundary = B{k};
%plot(boundary(:,2), boundary(:,1), 'r','LineWidth',2);

%for l=find(A(:,k))
%boundary = B{l};
%plot(boundary(:,2), boundary(:,1), 'g','LineWidth',2);
%end

%end

%end

%Display the background substraction of the original image


% --- Executes on button press in Object_Detection.
function Object_Detection_Callback(hObject, eventdata, handles)
% hObject    handle to Object_Detection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
boxImage=handles.b;
%boxImage = imread('mouse.jpg');
%figure;
%imshow(boxImage);
%title('Image of an Object');
sceneImage=handles.c;
%sceneImage = imread('scene.jpg');
%figure;
%imshow(sceneImage);
%title('Image of a Scene');
boxPoints = detectSURFFeatures(rgb2gray(boxImage));
scenePoints = detectSURFFeatures(rgb2gray(sceneImage));
%figure;
%imshow(boxImage);
%title('300 Strongest Feature Points from Box Image');
%hold on;
%plot(selectStrongest(boxPoints, 300));
%figure;
%imshow(sceneImage);
%title('300 Strongest Feature Points from Scene Image');
%hold on;
%plot(selectStrongest(scenePoints, 300));
[boxFeatures, boxPoints] = extractFeatures(rgb2gray(boxImage), boxPoints);
[sceneFeatures, scenePoints] = extractFeatures(rgb2gray(sceneImage), scenePoints);
boxPairs = matchFeatures(boxFeatures, sceneFeatures);
matchedBoxPoints = boxPoints(boxPairs(:, 1), :);
matchedScenePoints = scenePoints(boxPairs(:, 2), :);
%figure;
showMatchedFeatures(boxImage, sceneImage, matchedBoxPoints, matchedScenePoints, 'montage');
title('Putatively Matched Points (Including Outliers)');
[tform, inlierBoxPoints, inlierScenePoints] = estimateGeometricTransform(matchedBoxPoints, matchedScenePoints, 'similarity');
figure;
showMatchedFeatures(boxImage, sceneImage, inlierBoxPoints, inlierScenePoints, 'montage');
title('Matched Points (Inliers Only)');
boxPolygon = [1, 1;...                           % top-left
        size(boxImage, 2), 1;...                 % top-right
        size(boxImage, 2), size(boxImage, 1);... % bottom-right
        1, size(boxImage, 1);...                 % bottom-left
        1, 1];                   % top-left again to close the polygon

newBoxPolygon = transformPointsForward(tform, boxPolygon);
figure;
 axes(handles.axes3);
imshow(sceneImage);
hold on;
line(newBoxPolygon(:, 1), newBoxPolygon(:, 2), 'Color', 'y');
title('Detected Object');


% --- Executes on button press in Human_Motion_Detection.
function Human_Motion_Detection_Callback(hObject, eventdata, handles)
% hObject    handle to Human_Motion_Detection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



video = VideoReader('cut4.mp4'); 
nframes=video.NumberOfFrames;

for i=1:nframes
mov(i).cdata=read(video,i) 
end 

temp = zeros(size(mov(1).cdata));
[M,N] = size(temp(:,:,1));

for i = 1:10
temp = double(mov(i).cdata) + temp;
end

imbkg = temp/10;
centroidx = zeros(nframes,1);
centroidy = zeros(nframes,1);
predicted = zeros(nframes,4);
actual = zeros(nframes,4);

R=[[0.2845,0.0045]',[0.0045,0.0455]'];
H=[[1,0]',[0,1]',[0,0]',[0,0]'];
Q=0.01*eye(4);
P = 100*eye(4);
dt=1;
A=[[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];
kfinit = 0;
th = 38;

for i=1:nframes
 axes(handles.axes3);
imshow(mov(i).cdata);
hold on
imcurrent = double(mov(i).cdata);
diffimg = zeros(M,N);
diffimg = (abs(imcurrent(:,:,1)-imbkg(:,:,1))>th) ...
| (abs(imcurrent(:,:,2)-imbkg(:,:,2))>th) ...
| (abs(imcurrent(:,:,3)-imbkg(:,:,3))>th);
labelimg = bwlabel(diffimg,4);
markimg = regionprops(labelimg,['basic']);
[MM,NN] = size(markimg);
for nn = 1:MM
if markimg(nn).Area > markimg(1).Area
tmp = markimg(1);
markimg(1)= markimg(nn);
markimg(nn)= tmp;
end
end

bb = markimg(1).BoundingBox;
xcorner = bb(1);
ycorner = bb(2);
xwidth = bb(3);
ywidth = bb(4);
cc = markimg(1).Centroid;
centroidx(i)= cc(1);
centroidy(i)= cc(2);
hold on
rectangle('Position',[xcorner ycorner xwidth ywidth],'EdgeColor','b');
hold on
plot(centroidx(i),centroidy(i), 'bx');
kalmanx = centroidx(i)- xcorner;
kalmany = centroidy(i)- ycorner;

if kfinit == 0
	predicted =[centroidx(i),centroidy(i),0,0]' ;
else
	predicted = A*actual(i-1,:)';
end

kfinit = 1;
Ppre = A*P*A' + Q;
K = Ppre*H'/(H*Ppre*H'+R);
actual(i,:) = (predicted + K*([centroidx(i),centroidy(i)]' - H*predicted))';
P = (eye(4)-K*H)*Ppre;
hold on
rectangle('Position',[(actual(i,1)-kalmanx)...
(actual(i,2)-kalmany) xwidth ywidth],'EdgeColor','r','LineWidth',1.5);
hold on
plot(actual(i,1),actual(i,2), 'rx','LineWidth',1.5);
drawnow;
end






% --- Executes on button press in Suspicious_Activity_Detection.
function Suspicious_Activity_Detection_Callback(hObject, eventdata, handles)
% hObject    handle to Suspicious_Activity_Detection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
bi=cell(1,5);
si=cell(1,15);

mail = 'matlab_2013@yahoo.com';
psswd = 'sendemail';
host = 'smtp.mail.yahoo.com';
port  = '465';
emailto = 'ash.gunde09@gmail.com';
m_subject = 'subject';
m_text = 'test';
setpref( 'Internet','E_mail', mail );
setpref( 'Internet', 'SMTP_Server', host );
setpref( 'Internet', 'SMTP_Username', mail );
setpref( 'Internet', 'SMTP_Password', psswd );
props = java.lang.System.getProperties;
props.setProperty( 'mail.smtp.user', mail );
props.setProperty( 'mail.smtp.host', host );
props.setProperty( 'mail.smtp.port', port );
props.setProperty( 'mail.smtp.starttls.enable', 'true' );
props.setProperty( 'mail.smtp.debug', 'true' );
props.setProperty( 'mail.smtp.auth', 'true' );
props.setProperty( 'mail.smtp.socketFactory.port', port );
props.setProperty( 'mail.smtp.socketFactory.class', 'javax.net.ssl.SSLSocketFactory' );
props.setProperty( 'mail.smtp.socketFactory.fallback', 'true' );
%sendmail(emailto , 'ALERT', 'Some kind of suspicious activity has been detected in the department lab!');

for k=1:5
for i=1:15
si{i} = imread(sprintf('frame%d.jpg',i));
bi{k} = imread(sprintf('%d.jpg',k));
boxPoints = detectSURFFeatures(rgb2gray(bi{k}));
scenePoints = detectSURFFeatures(rgb2gray(si{i}));
[boxFeatures, boxPoints] = extractFeatures(rgb2gray(bi{k}), boxPoints);
[sceneFeatures, scenePoints] = extractFeatures(rgb2gray(si{i}), scenePoints);
boxPairs = matchFeatures(boxFeatures, sceneFeatures);
matchedBoxPoints = boxPoints(boxPairs(:, 1), :);
matchedScenePoints = scenePoints(boxPairs(:, 2), :);
[tform, inlierBoxPoints, inlierScenePoints, status] = estimateGeometricTransform(matchedBoxPoints, matchedScenePoints, 'projective');
%d=size(boxPairs,1);
if(status==1)
ii=si{i};
if(k==1)
	img=imread('1.jpg');
	img1=rgb2gray(img);
	img2=im2bw(img1,graythresh(img1));
	img2=~img2;
	B = bwboundaries(img2);
	b=length(B);
	jj=imcrop(ii,[155 120 65 80]);
	im1=rgb2gray(jj);
	im2=im2bw(im1,graythresh(im1));
	im2=~im2;
	C = bwboundaries(im2);
	c=length(C);
	if(c<b)
		%sound(randn(3190, 1), 8192);
		kk=imfuse(img,jj,'falsecolor');
		imshow(kk);
		%sendmail(emailto , 'ALERT', 'Some kind of suspicious activity has been detected in the department lab!');
	end

elseif(k==2)
	img=imread('2.jpg');
	img1=rgb2gray(img);
	img2=im2bw(img1,graythresh(img1));
	img2=~img2;
	B = bwboundaries(img2);
	b=length(B);
	jj=imcrop(ii,[190 110 50 65]);
	im1=rgb2gray(jj);
	im2=im2bw(im1,graythresh(im1));
	im2=~im2;
	C = bwboundaries(im2);
	c=length(C);
	if(c<b)
		%sound(randn(3190, 1), 8192);
		kk=imfuse(img,jj,'falsecolor');
		imshow(kk);
		%sendmail(emailto , 'ALERT', 'Some kind of suspicious activity has been detected in the department lab!');
	end

elseif(k==3)
	img=imread('3.jpg');
	img1=rgb2gray(img);
	img2=im2bw(img1,graythresh(img1));
	img2=~img2;
	B = bwboundaries(img2);
	b=length(B);
	jj=imcrop(ii,[230 80 60 50]);
	im1=rgb2gray(jj);
	im2=im2bw(im1,graythresh(im1));
	im2=~im2;
	C = bwboundaries(im2);
	c=length(C);
	if(c<b)
		sound(randn(3190, 1), 8192);
		kk=imfuse(img,jj);
		imshow(kk);
		sendmail(emailto , 'ALERT', 'Some kind of suspicious activity has been detected in the department lab!');
	end

elseif(k==4)
	img=imread('4.jpg');
	img1=rgb2gray(img);
	img2=im2bw(img1,graythresh(img1));
	img2=~img2;
	B = bwboundaries(img2);
	b=length(B);
	jj=imcrop(ii,[260 50 50 60]);
	im1=rgb2gray(jj);
	im2=im2bw(im1,graythresh(im1));
	im2=~im2;
	C = bwboundaries(im2);
	c=length(C);
	if(c<b)
		sound(randn(3190, 1), 8192);
		kk=imfuse(img,jj,'falsecolor');
		imshow(kk);
		%sendmail(emailto , 'ALERT', 'Some kind of suspicious activity has been detected in the department lab!');
	end

elseif(k==5)
	img=imread('5.jpg');
	img1=rgb2gray(img);
	img2=im2bw(img1,graythresh(img1));
	img2=~img2;
	B = bwboundaries(img2);
	b=length(B);
	jj=imcrop(ii,[290 40 65 50]);
	im1=rgb2gray(jj);
	im2=im2bw(im1,graythresh(im1));
	im2=~im2;
	C = bwboundaries(im2);
	c=length(C);
	if(c<b)
		sound(randn(3190, 1), 8192);
		kk=imfuse(img,jj,'falsecolor');
        axes(handles.axes3);
		imshow(kk);
		sendmail(emailto , 'ALERT', 'Some kind of suspicious activity has been detected in the department lab!');
	end
end
end
end
end


% --- Executes on button press in Load_Image_1.
function Load_Image_1_Callback(hObject, eventdata, handles)
% hObject    handle to Load_Image_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename,pathname] = uigetfile('*.m','pick an m file');
if isequal(filename,0)|| isequal(pathname,0)
    disp('User pressed cancel')
else 
    filename = strcat(pathname,filename);
    b = imread(filename);
    axes(handles.axes1);
    imshow(b);
    handles.b=b;
    guidata(hObject,handles);
    %set(handles.text1,'String',filename);
end



% --- Executes on button press in Load_Image_2.
function Load_Image_2_Callback(hObject, eventdata, handles)
% hObject    handle to Load_Image_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename,pathname] = uigetfile('*.m','pick an m file');
if isequal(filename,0)|| isequal(pathname,0)
    disp('User pressed cancel')
else 
    filename = strcat(pathname,filename);
    c = imread(filename);
    axes(handles.axes2);
    imshow(c);
    handles.c=c;
    guidata(hObject,handles);
    %set(handles.text1,'String',filename);
end


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1


% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Image_Enhancement.
function Image_Enhancement_Callback(hObject, eventdata, handles)
% hObject    handle to Image_Enhancement (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Image_Enhancement contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Image_Enhancement
val =get(hObject,'Value');
str=get(hObject,'String');
switch str(val)
    case 'Histogram_Equalisation'
        handles.current_data=handles.Histogram_Equalisation;
    case 'Weiner_Filter'
        handles.current_data=handles.Weiner_Filter;
    case 'Unsharp_Mask_Filtering'
        handles.current_data=handles.Unsharp_Mask_Filtering;
        
end
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function Image_Enhancement_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Image_Enhancement (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
I=handles.b;
I = im2double(imread('001.jpg'));

%imshow(I);
title('Original Image (courtesy of MIT)');
 LEN = 21;
THETA = 11;
PSF = fspecial('motion', LEN, THETA);
blurred = imfilter(I, PSF, 'conv', 'circular');
imshow(blurred);
title('Blurred Image');
 wnr1 = deconvwnr(blurred, PSF, 0);
imshow(wnr1);
title('Restored Image');
I = im2double(imread('001.jpg'));
imshow(I);
title('Original Image (courtesy of MIT)');
 LEN = 21;
THETA = 11;
PSF = fspecial('motion', LEN, THETA);
blurred = imfilter(I, PSF, 'conv', 'circular');
    axes(handles.axes2);
imshow(blurred);
title('Blurred Image');
 wnr1 = deconvwnr(blurred, PSF, 0);
  axes(handles.axes3);
imshow(wnr1);
title('Restored Image');


% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Feature_Extraction.
function Feature_Extraction_Callback(hObject, eventdata, handles)
% hObject    handle to Feature_Extraction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Feature_Extraction contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Feature_Extraction
val =get(hObject,'Value');
str=get(hObject,'String');
switch str(val)
    case 'SURF'
        handles.current_data=handles.SURF;
    case 'FAST'
        handles.current_data=handles.FAST;
    case 'Harris'
        handles.current_data=handles.FAST;
        
end
guidata(hObject,handles);
        


% --- Executes during object creation, after setting all properties.
function Feature_Extraction_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Feature_Extraction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when figure1 is resized.
function figure1_ResizeFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
