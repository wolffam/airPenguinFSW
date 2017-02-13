%% Begin %%
function varargout = Rocket_GUI_Interface(varargin)
% ROCKET_GUI_INTERFACE MATLAB code for Rocket_GUI_Interface.fig
%      ROCKET_GUI_INTERFACE, by itself, creates a new ROCKET_GUI_INTERFACE or raises the existing
%      singleton*.
%
%      H = ROCKET_GUI_INTERFACE returns the handle to a new ROCKET_GUI_INTERFACE or the handle to
%      the existing singleton*.
%
%      ROCKET_GUI_INTERFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROCKET_GUI_INTERFACE.M with the given input arguments.
%
%      ROCKET_GUI_INTERFACE('Property','Value',...) creates a new ROCKET_GUI_INTERFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Rocket_GUI_Interface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Rocket_GUI_Interface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Rocket_GUI_Interface

% Last Modified by GUIDE v2.5 21-May-2016 16:19:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Rocket_GUI_Interface_OpeningFcn, ...
                   'gui_OutputFcn',  @Rocket_GUI_Interface_OutputFcn, ...
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


%% --- Executes just before Rocket_GUI_Interface is made visible. %%
function Rocket_GUI_Interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Rocket_GUI_Interface (see VARARGIN)

% Define SKETCHY AS FUCK global variables to allow plotting to occur in the
% background

% Incrementer variable for plot 1
global x1;
x1 = 0;
% Incrementer variable for plot 2
global x2;
x2 = 0;
% Array to hold Pressure1 values for plot 1
global y1;
y1 = [];
% Array to hold Pressure2 values for plot 1
global y12;
y12 = [];
% Array to hold Pressure3 values for plot 1
global y13;
y13 = [];
% Array to hold values for plot 2
global y2;
y2 = [];
% Array to hold time for plot 1
global time1;
time1 = [];
% Array to hold time for plot 2
global time2;
time2 = [];
% Axes for plot 1
global axes1;
axes1 = handles.axes1;
global axes2;
% Axes for plot 2
axes2 = handles.axes2;
% Timer for plot 1
global t1;
% Timer for plot 2
global t2;
% Timer for mission time
global t3;
% Mission Clock
global MissionClock;
MissionClock = 0;
global MissionText;
MissionText = handles.text9;
% The current state of the flight computer
global State;
State = 0;
global MissionState;
MissionState = handles.text11;
% Timer for data read
global t4;

% Create globals for all data that we are reading in
global Pressure1;
global Pressure12;
global Pressure13;
global Thrust;
Pressure1 = 0;
Pressure12 = 0;
Pressure13 = 0;
Thrust = 0;

% Create a global for the comport
global comport;

% Create a file for writing data to
fileID1 = fopen('RocketDataPressure.txt', 'w');
fprintf(fileID1, 'PCC (psi) PFOX (psi) PSOX (psi)   Time (s) \n');
fclose(fileID1);
fileID2 = fopen('RocketDataThrust.txt', 'w');
fprintf(fileID2, 'Thrust (lbs) Time (s) \n');
fclose(fileID2);

% Then open serial port to teensy
%comport = serial('/dev/cu.usbmodem1426161', 'BaudRate', 9600); % setup a serial port for communication (ground teensy-David)
% comport = serial('/dev/cu.usbmodem1501061', 'BaudRate', 9600); % setup a serial port for communication (flight teensy-David)
comport = serial('Com8', 'BaudRate', 9600); % setup a serial port (ground teensy - Brian)
fopen(comport); % open comport

% Initialize Valve 1 status
V1 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V1.Position = [167 540 50 15];

% Initialize Valve 2 status
V2 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V2.Position = [398 540 50 15];

% Initialize Valve 3 status
V3 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V3.Position = [631 540 50 15];

% Initialize Valve 4 status
V4 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V4.Position = [866 540 50 15];

% Initialize Valve 5 status
V5 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V5.Position = [167 469 50 15];

% Initialize Valve 6 status
V6 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V6.Position = [398 469 50 15];

% Initialize Valve 7 status
V7 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V7.Position = [631 469 50 15];

% Initialize Valve 8 status
V8 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V8.Position = [866 469 50 15];


% Initialize plot 1
axes(handles.axes1);
grid on;
tic;

% Initialize plot 2
axes(handles.axes2);
grid on;    

% Create a timer object to handle background plotting queries

t1 = timer('TimerFcn', @BackgroundPlot1, 'ExecutionMode', 'fixedRate', 'TasksToExecute', Inf, 'Period', 0.5);

t2 = timer('TimerFcn', @BackgroundPlot2, 'ExecutionMode', 'fixedRate', 'TasksToExecute', Inf, 'Period', 0.5);

% Create a timer object to update mission clock
t3 = timer('TimerFcn', @MissionTime, 'ExecutionMode', 'fixedRate', 'TasksToExecute', Inf, 'Period', 0.1);
start(t3);

% Create a timer object to handle data acquisition
t4 = timer('TimerFcn', @MissionData, 'ExecutionMode', 'fixedRate', 'TasksToExecute', Inf, 'Period', 0.1);
start(t4);

% Choose default command line output for Rocket_GUI_Interface
handles.output = hObject;

handles.comport = comport;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Rocket_GUI_Interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%% Handler of background plotting for pressure %%
function varargout = BackgroundPlot1(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Load SKETCHY AS FUCK global variables for plot 1
global x1;
global y1;
global y12;
global y13;
global Pressure1;
global Pressure12;
global Pressure13;
global time1;
global axes1;

% Get the time that has elapsed since gui was opened
time1 = [time1 toc];

% Append next value to be plotted in plot 1
%new = rand();

y1 = [y1 Pressure1];
y12 = [y12 Pressure12];
y13 = [y13 Pressure13];


% Update plot 1
plot(axes1, time1, [y1; y12; y13]);
xlim(axes1,[toc-10 toc]);
legend(axes1,'PCC','PFOX','PSOX','Location','northwest');
grid(axes1,'on');

% Increment counter 
x1 = x1 + 1;

%% Handler of background plotting for thrust %%
function varargout = BackgroundPlot2(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Load SKETCHY AS FUCK global variables for plot 2
global x2;
global y2;
global time2;
global axes2;
global Thrust;

% Get the time that has elapsed since gui was opened
time2 = [time2 toc];

% Append next value to be plotted in plot 2
%new = rand();
y2 = [y2 Thrust];

% Update plot 2
plot(axes2,time2, y2);
xlim(axes2,[toc-10 toc]);
grid(axes2,'on');

% Increment counter
x2 = x2 + 1;

% --- Outputs from this function are returned to the command line.
function varargout = Rocket_GUI_Interface_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


%% Plot Pressure %%
% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global t1;

start(t1);

%% End Pressure Plot %%
% --- Executes on button press in pushbutton28.
function pushbutton28_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global t1;
stop(t1);

%handles.stop_now = 1;
%guidata(hObject, handles);  % Update guidata

%% Clear pressure plot %%
% --- Executes on button press in pushbutton33.
function pushbutton33_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global time1;
global axes1;
global x1;
global y1;
global y12;
global y13;

time1 = [];
x1 = 0;
y1 = [];
y12 = [];
y13 = [];

cla(axes1);
grid(axes1,'on');

%% Plot Thrust %%
% --- Executes on button press in pushbutton30.
function pushbutton30_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global t2;
start(t2);

comport = handles.comport;
fprintf(comport,'u');

%% End Plot Thrust %%
% --- Executes on button press in pushbutton31.
function pushbutton31_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global t2;
stop(t2);

%% Clear thrust plot %%
% --- Executes on button press in pushbutton34.
function pushbutton34_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global time2;
global axes2;
global x2;
global y2;

time2 = [];
x2 = 0;
y2 = [];

cla(axes2);
grid(axes2,'on');

comport = handles.comport;

fprintf(comport,'u');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Begin Valves %

%% ACT-SVV push button 'ON'%%
% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

V1 = uicontrol('style','text','backgroundcolor',[0 1 0], 'string','OPEN','foregroundcolor',[0 0 0]);
V1.Position = [167 540 50 15];

disp('SVV-ON');
comport = handles.comport;

fprintf(comport,'a');

% ACT-SVV push button 'OFF'%
% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

V1 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V1.Position = [167 540 50 15];

disp('SVV-OFF');
comport = handles.comport;

fprintf(comport,'b');

%% ACT-VSOX push button 'ON'%%
% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V2 = uicontrol('style','text','backgroundcolor',[0 1 0], 'string','OPEN','foregroundcolor',[0 0 0]);
V2.Position = [398 540 50 15];

comport = handles.comport;

fprintf(comport,'c');

% ACT-VSOX push button 'OFF'%
% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V2 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V2.Position = [398 540 50 15];

comport = handles.comport;

fprintf(comport,'d');

%% ACT-NPV push button 'ON'%%
% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V3 = uicontrol('style','text','backgroundcolor',[0 1 0], 'string','OPEN','foregroundcolor',[0 0 0]);
V3.Position = [631 540 50 15];

comport = handles.comport;

fprintf(comport,'e');

% ACT-NPV push button 'OFF'%
% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V3 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V3.Position = [631 540 50 15];

comport = handles.comport;

fprintf(comport,'f');

%% Valve 4 push button 'ON'%%
% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V4 = uicontrol('style','text','backgroundcolor',[0 1 0], 'string','OPEN','foregroundcolor',[0 0 0]);
V4.Position = [866 540 50 15];

comport = handles.comport;

fprintf(comport,'g');

% Valve 4 push button 'OFF'%
% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V4 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V4.Position = [866 540 50 15];

comport = handles.comport;

fprintf(comport,'h');

%% Valve 5 push button 'ON'%%
% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V5 = uicontrol('style','text','backgroundcolor',[0 1 0], 'string','OPEN','foregroundcolor',[0 0 0]);
V5.Position = [167 469 50 15];

comport = handles.comport;

fprintf(comport,'i');

% Valve 5 push button 'OFF'%
% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V5 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V5.Position = [167 469 50 15];

comport = handles.comport;

fprintf(comport,'j');

%% Valve 6 push button 'ON'%%
% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V6 = uicontrol('style','text','backgroundcolor',[0 1 0], 'string','OPEN','foregroundcolor',[0 0 0]);
V6.Position = [398 469 50 15];

comport = handles.comport;

fprintf(comport,'k');

% Valve 6 push button 'OFF'%
% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V6 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V6.Position = [398 469 50 15];

comport = handles.comport;

fprintf(comport,'l');


%% ACT-SFOXV push button 'ON'%%
% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V7 = uicontrol('style','text','backgroundcolor',[0 1 0], 'string','OPEN','foregroundcolor',[0 0 0]);
V7.Position = [631 469 50 15];

comport = handles.comport;

fprintf(comport,'m');

% ACT-SFOXV push button 'OFF'%
% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V7 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V7.Position = [631 469 50 15];

comport = handles.comport;

fprintf(comport,'n');

%% Valve 8 push button 'ON'%%
% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V8 = uicontrol('style','text','backgroundcolor',[0 1 0], 'string','OPEN','foregroundcolor',[0 0 0]);
V8.Position = [866 469 50 15];

comport = handles.comport;

fprintf(comport,'o');

% Valve 8 push button 'OFF'%
% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
V8 = uicontrol('style','text','backgroundcolor',[1 0 0], 'string','CLOSED','foregroundcolor',[0 0 0]);
V8.Position = [866 469 50 15];

comport = handles.comport;

fprintf(comport,'p');

% End Valves %

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Hard Abort %%
% --- Executes on button press in pushbutton26.
function pushbutton26_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

comport = handles.comport;

fprintf(comport,'r');

%% Soft Abort %%
% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

comport = handles.comport;

fprintf(comport,'s');

%% Launch %%
% --- Executes on button press in pushbutton27.
function pushbutton27_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

display('In Launch');


comport = handles.comport;

fprintf(comport,'t');

%% Clean up GUI envirmonment and then close GUI %%
% --- Executes on button press in pushbutton32.
function pushbutton32_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Load timers for plots 1 and 2
global t1;
global t2;
global t3;
global t4;

% Load data to be printed
global y1;
global y12;
global y13;
global y2;
global time1;
global time2;

% Print all data to text files
data1 = y1;
data12 = y12;
data13 = y13;
fileID1 = fopen('RocketDataPressure.txt', 'a');
fprintf(fileID1,'%12.8d %12.8d %12.8d %12.8d \n', [data1; data12; data13; time1]);
fclose(fileID1);

data = y2;
fileID2 = fopen('RocketDataThrust.txt', 'a');
fprintf(fileID2,'%12.8e %12.8e \n', [data; time2]);
fclose(fileID2);

% Close all serial ports 
fclose(instrfind);

% End all timers
stop(t1);
stop(t2);
stop(t3);
stop(t4);

% Delete all timers
delete(t1);
delete(t2);
delete(t3);
delete(t4);

% Clear terminal
clear all
clc

% Tell user that GUI is closing
disp('Exiting Rocket GUI');

pause(1);

% Clear Globals
clear global;

% Close everything
close all;

% Tell user that GUI has closed succesfully
disp('Rocket GUI Exit Successful');

%% Mission Clock %%
% --- Executes on button press in pushbutton1.
function MissionTime(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global MissionClock;
global MissionText;

MissionClock = toc;
set(MissionText, 'String', num2str(MissionClock));

%% Mission Data %%
% --- Executes on button press in pushbutton1.
function MissionData(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Pressure1;
global Pressure12;
global Pressure13;
global Thrust;
global State;
global MissionState;
global comport

% Read in data
%comport.BytesAvailable
if comport.BytesAvailable > 0
    Pressure1_tmp = fscanf(comport, '%f');  % receive digital input;
    Pressure12_tmp = fscanf(comport, '%f');  % receive digital input;
    Pressure13_tmp = fscanf(comport, '%f');  % receive digital input;
    
    Pressure1 = 600*(Pressure1_tmp*5.0/1023.0)/(4.58 - 2.19) - 249.791;    % PCC
    Pressure12 = 600*(Pressure12_tmp*5.0/1023.0)/(4.58 - 2.19) - 249.791;  % PFOX
    Pressure13 = 300*(Pressure13_tmp*5.0/1023.0)/(2.72 - 2.11) - 737.705;  % PSOX
    
    fprintf('PCC = %f psi\n',Pressure1_tmp);
    fprintf('PFOX = %f psi\n',Pressure12_tmp);
    fprintf('PSOX = %f psi\n\n',Pressure13_tmp);
    
    Thrust = fscanf(comport, '%f');  % receive digital input;
    State = fscanf(comport, '%f');  % receive digital input;
end

%Pressure1 = rand();
%Pressure12 = rand();

set(MissionState, 'String', num2str(State));

