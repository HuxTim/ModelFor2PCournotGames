function varargout = CournotGUI(varargin)
%GUI initialization
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @CournotGUI_OpeningFcn, ...
    'gui_OutputFcn',  @CournotGUI_OutputFcn, ...
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


% Executes just before CournotGUI is made visible.
function CournotGUI_OpeningFcn(hObject, eventdata, handles, varargin)
global data;
global p1payoffsVector;
global p2payoffsVector;
global outerBoundaryPoints;

handles.output = hObject;% Choose default command line output for CournotGUI
guidata(hObject, handles);% Update handles structure

%% Main Algorithm
p1payoffs = [16 3 0;21 10 -1;9 5 -5];
p2payoffs = [9 13 3;1 4 0;0 -4 -15];
discountRate = .3;
p1payoffsVector = reshape(p1payoffs,1,numel(p1payoffs))'; %x
p2payoffsVector = reshape(p2payoffs,1,numel(p2payoffs))'; %y
validIntersections = [];
outerBoundaryPoints = boundary(p1payoffsVector,p2payoffsVector,0);%boundary points
bound1 = outerBoundaryPoints;
%Set iteration number
iterations = 3;
%Create a cell array to store all the varialbes in the end of the loop
data = cell(1, iterations);
polygon = [p1payoffsVector(outerBoundaryPoints),p2payoffsVector(outerBoundaryPoints)];
%For loop to go through each iteration
for i=1:iterations
    %Find the intersections points by calling findIntersection points
    validIntersections = findIntersections(p1payoffs,p2payoffs,polygon,discountRate);
    tempP1 = p1payoffsVector;%temporary payoff vector
    tempP2 = p2payoffsVector;%temporary payoff vector
    newexpointX=[];
    newexpointY=[];
    %Compute new extreme points using the algorithm
    newexpointX = discountRate*validIntersections(:,1)+(1-discountRate)*validIntersections(:,3);
    newexpointY = discountRate*validIntersections(:,2)+(1-discountRate)*validIntersections(:,4);
    %Store continuation values by calling operator and findsquareIntersect
    %function
    [w1 w2] = operator(p1payoffs,p2payoffs,discountRate);
    [w3 w4] = findsquareIntersect(p1payoffs,p2payoffs,discountRate);
    
    %Search for the points that are not included in the operator
    index=find(w1==0 & w2==0);
    w3new = reshape(w3,1,numel(w3))';
    w4new = reshape(w4,1,numel(w4))';
    w3new(index)=[];
    w4new(index)=[];
    tempP1(index)=[];
    tempP2(index)=[];
    %Search for the points that are intersecting with each other
    in = inpolygon(w3new,w4new,polygon(:,1),polygon(:,2));
    index2=find(in==0);
    w3new(index2)=[];
    tempP1(index2)=[];
    w4new(index2)=[];
    tempP2(index2)=[];
    %Store vertext intersection points coordinates
    w3final = [];
    w4final = [];
    w3final(:,1)=w3new;
    w3final(:,2)=tempP1;
    w4final(:,1)=w4new;
    w4final(:,2)=tempP2;
    %compute new extreme points by using the revised points
    newexpointX = [newexpointX; (1-discountRate)*tempP1+(discountRate)*w3new];
    newexpointY = [newexpointY; (1-discountRate)*tempP2+(discountRate)*w4new];
    bound1 = boundary(newexpointX,newexpointY, 0);
    polygon = [newexpointX(bound1),newexpointY(bound1)];
    %Separating each valid intersections and vertex intersections into cell
    %array.
    vi=cell(2,6);
    vi{1,1}=validIntersections(1,:);
    vi{1,2}=validIntersections(2,:);
    vi{1,3}=validIntersections(3:1:4,:);
    vi{1,4}=[];
    vi{1,5}=validIntersections(5:1:6,:);
    vi{1,6}=validIntersections(7:1:8,:);
    vi{2,1}=validIntersections(1,:);
    vi{2,2}=validIntersections(2:1:3,:);
    vi{2,3}=validIntersections(4:1:5,:);
    vi{2,4}=[];
    vi{2,5}=validIntersections(6:1:7,:);
    vi{2,6}=validIntersections(8:1:end,:);
    wf=cell(2,6);
    wf{1,1}=[w3final(1,:) w4final(1,:)];
    wf{1,2}=[];
    wf{1,3}=[];
    wf{1,4}=[];
    wf{1,5}=[];
    wf{1,6}=[w3final(size(w3final)-1,:) w4final(size(w3final)-1,:)];
    wf{2,1}=[w3final(1,:) w4final(1,:)];
    wf{2,2}=[];
    wf{2,3}=[];
    wf{2,4}=[];
    wf{2,5}=[];
    wf{2,6}=[w3final(2,:) w4final(2,:)];
    %Store all the variables needed in the plotting and in the other
    %functions into cell array
    iterationData = {validIntersections, w3, w4, newexpointX, newexpointY, bound1, vi, wf, w3final, w4final};
    data{i} = iterationData;
    
end
%Plot the palyer payoff options
plot(p1payoffsVector,p2payoffsVector, 'ko','MarkerSize', 4.5)
%Control axis scaling
axis([min(p1payoffsVector)-5,max(p1payoffsVector)+5,min(p2payoffsVector)-5,max(p2payoffsVector)+5]);
hold on;
%Plot the feasible payoff set
plot(p1payoffsVector(outerBoundaryPoints),p2payoffsVector(outerBoundaryPoints), 'r');
hold on;


% Outputs from this function are returned to the command line.
function varargout = CournotGUI_OutputFcn(hObject, eventdata, handles)
% Get default command line output from handles structure
varargout{1} = handles.output;

% Executes on button press in radio11.
function radio11_Callback(hObject, eventdata, handles)
global data;
global iteration;
global sliderValue;
% get statement to check if the button is clicked
if(get(handles.radio11,'Value'))
    if sliderValue>=0 && sliderValue<=1
        v=line([data{iteration}{7}{1,1}(1,1),data{iteration}{7}{1,1}(1,3)],[data{iteration}{7}{1,1}(1,2),data{iteration}{7}{1,1}(1,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        w=line([data{iteration}{8}{1,1}(1),data{iteration}{8}{1,1}(2)],[data{iteration}{8}{1,1}(3),data{iteration}{8}{1,1}(4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        p1=plot(data{iteration}{7}{1,1}(:,1),data{iteration}{7}{1,1}(:,2),'m*')
        p2=plot(data{iteration}{8}{1,1}(:,1),data{iteration}{8}{1,1}(:,3),'m*')
        hold off;
    elseif sliderValue>1
        v=line([data{iteration}{7}{2,1}(1,1),data{iteration}{7}{2,1}(1,3)],[data{iteration}{7}{2,1}(1,2),data{iteration}{7}{2,1}(1,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        w=line([data{iteration}{8}{1,1}(1),data{iteration}{8}{1,1}(2)],[data{iteration}{8}{1,1}(3),data{iteration}{8}{1,1}(4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        p1=plot(data{iteration}{7}{2,1}(:,1),data{iteration}{7}{2,1}(:,2),'m*')
        p2=plot(data{iteration}{8}{1,1}(:,1),data{iteration}{8}{1,1}(:,3),'m*')
        hold off;
    end
else
    delete(v);
    delete(w);
    delete(p1);
    delete(p2);
end

% Executes on button press in radio12.
function radio12_Callback(hObject, eventdata, handles)
global data;
global iteration;
global sliderValue;
% Check if the button is clicked
if(get(handles.radio12,'Value'))
    if sliderValue>=0 && sliderValue<=1
        v=line([data{iteration}{7}{1,2}(1,1),data{iteration}{7}{1,2}(1,3)],[data{iteration}{7}{1,2}(1,2),data{iteration}{7}{1,2}(1,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        p1=plot(data{iteration}{7}{1,2}(:,1),data{iteration}{7}{1,2}(:,2),'m*')
        hold off;
    elseif sliderValue > 1
        v=line([data{iteration}{7}{2,2}(1,1),data{iteration}{7}{2,2}(1,3)],[data{iteration}{7}{2,2}(1,2),data{iteration}{7}{2,2}(1,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        v2=line([data{iteration}{7}{2,2}(2,1),data{iteration}{7}{2,2}(2,3)],[data{iteration}{7}{2,2}(2,2),data{iteration}{7}{2,2}(2,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        p1=plot(data{iteration}{7}{2,2}(:,1),data{iteration}{7}{2,2}(:,2),'m*')
        hold off;
    end
else
    delete(v);
    delete(v2);
    delete(p1);
end

% --- Executes on button press in radio21.
function radio21_Callback(hObject, eventdata, handles)
global data;
global iteration;
global sliderValue;
% get statement to check if the button is clicked
if(get(handles.radio21,'Value'))
    if sliderValue>=0 && sliderValue<=1
        v=line([data{iteration}{7}{1,3}(1,1),data{iteration}{7}{1,3}(1,3)],[data{iteration}{7}{1,3}(1,2),data{iteration}{7}{1,3}(1,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        v2=line([data{iteration}{7}{1,3}(2,1),data{iteration}{7}{1,3}(2,3)],[data{iteration}{7}{1,3}(2,2),data{iteration}{7}{1,3}(2,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        p1=plot(data{iteration}{7}{1,3}(:,1),data{iteration}{7}{1,3}(:,2),'m*')
        hold off;
    elseif sliderValue > 1
        v=line([data{iteration}{7}{2,3}(1,1),data{iteration}{7}{2,3}(1,3)],[data{iteration}{7}{2,3}(1,2),data{iteration}{7}{2,3}(1,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        v2=line([data{iteration}{7}{2,3}(2,1),data{iteration}{7}{2,3}(2,3)],[data{iteration}{7}{2,3}(2,2),data{iteration}{7}{2,3}(2,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        p1=plot(data{iteration}{7}{2,3}(:,1),data{iteration}{7}{2,3}(:,2),'m*')
        hold off;
    end
else
    delete(v);
    delete(v2);
    delete(p1);
end

% --- Executes on button press in radio21.
function radio22_Callback(hObject, eventdata, handles)

% --- Executes on button press in radio23.
function radio23_Callback(hObject, eventdata, handles)
global data;
global iteration;
global sliderValue;
% get statement to check if the button is clicked
if(get(handles.radio23,'Value'))
    if sliderValue>=0 && sliderValue<=1
        v1=line([data{iteration}{7}{1,5}(1,1),data{iteration}{7}{1,5}(1,3)],[data{iteration}{7}{1,5}(1,2),data{iteration}{7}{1,5}(1,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        v2=line([data{iteration}{7}{1,5}(2,1),data{iteration}{7}{1,5}(2,3)],[data{iteration}{7}{1,5}(2,2),data{iteration}{7}{1,5}(2,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        p1=plot(data{iteration}{7}{1,5}(:,1),data{iteration}{7}{1,5}(:,2),'m*')
        hold off;
    elseif sliderValue>1
        v1=line([data{iteration}{7}{2,5}(1,1),data{iteration}{7}{2,5}(1,3)],[data{iteration}{7}{2,5}(1,2),data{iteration}{7}{2,5}(1,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        v2=line([data{iteration}{7}{2,5}(2,1),data{iteration}{7}{2,5}(2,3)],[data{iteration}{7}{2,5}(2,2),data{iteration}{7}{2,5}(2,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        p1=plot(data{iteration}{7}{2,5}(:,1),data{iteration}{7}{2,5}(:,2),'m*')
        hold off;
    end
else
    delete(v1);
    delete(v2);
    delete(p1);
end


% --- Executes on button press in radio32.
function radio32_Callback(hObject, eventdata, handles)
global data;
global iteration;
global sliderValue;
% get statement to check if the button is clicked
if(get(handles.radio32,'Value'))
    if sliderValue>=0 && sliderValue<=1
        v1=line([data{iteration}{7}{1,6}(1,1),data{iteration}{7}{1,6}(1,3)],[data{iteration}{7}{1,6}(1,2),data{iteration}{7}{1,6}(1,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        v2=line([data{iteration}{7}{1,6}(2,1),data{iteration}{7}{1,6}(2,3)],[data{iteration}{7}{1,6}(2,2),data{iteration}{7}{1,6}(2,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        w=line([data{iteration}{9}(3,1),data{iteration}{9}(3,2)],[data{iteration}{10}(3,1),data{iteration}{10}(3,2)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        p1=plot(data{iteration}{7}{1,6}(:,1),data{iteration}{7}{1,6}(:,2),'m*')
        p2=plot(data{iteration}{8}{1,6}(:,1),data{iteration}{8}{1,6}(:,3),'m*')
        hold off
    elseif sliderValue>1
        v1=line([data{iteration}{7}{2,6}(1,1),data{iteration}{7}{2,6}(1,3)],[data{iteration}{7}{2,6}(1,2),data{iteration}{7}{2,6}(1,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        v2=line([data{iteration}{7}{2,6}(2,1),data{iteration}{7}{2,6}(2,3)],[data{iteration}{7}{2,6}(2,2),data{iteration}{7}{2,6}(2,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        w=line([data{iteration}{8}{2,6}(1),data{iteration}{8}{2,6}(2)],[data{iteration}{8}{2,6}(3),data{iteration}{8}{2,6}(4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        p1=plot(data{iteration}{7}{2,6}(:,1),data{iteration}{7}{2,6}(:,2),'m*')
        p2=plot(data{iteration}{8}{2,6}(:,1),data{iteration}{8}{2,6}(:,3),'m*')
        hold off;
    end
else
    delete(v1);
    delete(v2);
    delete(w);
    delete(p1);
    delete(p2);
end

% --- Executes on slider movement.
function slider_Callback(hObject, eventdata, handles)
global data;
global p1payoffsVector;
global p2payoffsVector;
global outerBoundaryPoints;
global iteration;
global sliderValue;
% Set minumun and maximum value of the slider and the sliderstep.
maxIteration = 3;
set(handles.slider, 'Min', 0); % Set to 0.
set(handles.slider, 'Max', maxIteration); % Set to maximum iteration
%Each click on the arrow will move the slider 0.01 point; Each click on
%the bar will 0.1 point.
set(handles.slider, 'SliderStep', [0.01, 0.1]); 
%Set a slider value by useing the handles object
sliderValue = int32(get(handles.slider,'Value'));
%convert between the slider value and the string in the text box
set(handles.sliderStatus,'string',num2str(sliderValue));
guidata(hObject,handles);
if sliderValue >= 0 && sliderValue <= 1
    cla; %clear the figure each time the figure has to change.
    iteration = 1;
    plot(p1payoffsVector,p2payoffsVector, 'ko','MarkerSize', 4.5);
    axis([min(p1payoffsVector)-5,max(p1payoffsVector)+5,min(p2payoffsVector)-5,max(p2payoffsVector)+5]);
    hold on;
    plot(p1payoffsVector(outerBoundaryPoints),p2payoffsVector(outerBoundaryPoints), 'r');
    hold on;
    plot(data{1}{4},data{1}{5},'bo','MarkerSize', 4.5);
    hold on;
    plot(data{1}{4}(data{1}{6}),data{1}{5}(data{1}{6}), 'c');
    hold on; %leave it on for the user to click on the extreme points
elseif sliderValue > 1 && sliderValue <= 2
    cla;
    iteration = 2;
    plot(p1payoffsVector,p2payoffsVector, 'ko','MarkerSize', 4.5);
    axis([min(p1payoffsVector)-5,max(p1payoffsVector)+5,min(p2payoffsVector)-5,max(p2payoffsVector)+5]);
    hold on;
    plot(p1payoffsVector(outerBoundaryPoints),p2payoffsVector(outerBoundaryPoints), 'r');
    hold on;
    plot(data{2}{4},data{2}{5},'bo','MarkerSize', 4.5);
    hold on;
    plot(data{2}{4}(data{2}{6}),data{2}{5}(data{2}{6}), 'c');
    hold on;
elseif sliderValue > 2 && sliderValue <= 3
    cla;
    iteration = 3;
    plot(p1payoffsVector,p2payoffsVector, 'ko','MarkerSize', 4.5);
    axis([min(p1payoffsVector)-5,max(p1payoffsVector)+5,min(p2payoffsVector)-5,max(p2payoffsVector)+5]);
    hold on;
    plot(p1payoffsVector(outerBoundaryPoints),p2payoffsVector(outerBoundaryPoints), 'r');
    hold on;
    plot(data{3}{4},data{3}{5},'bo','MarkerSize', 4.5);
    hold on;
    plot(data{3}{4}(data{3}{6}),data{3}{5}(data{3}{6}), 'c');
    hold on;
else
    uiwait();    %For exception, uiwait
end
% Executes during object creation, after setting all properties.
function slider_CreateFcn(hObject, eventdata, handles)
% Set white background
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% Executes on text box call
function sliderStatus_Callback(hObject, eventdata, handles)
%Retrieve the slider value and convert into string.
editString=get(hObject,'string');
set(handles.slider,'value',str2num(editString));
guidata(hObject,handles);

% Executes during object creation, after setting all properties.
function sliderStatus_CreateFcn(hObject, eventdata, handles)
% Set white background
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% Executes on button press in showAllbutton.
function showAllbutton_Callback(hObject, eventdata, handles)
global data;
global iteration;
if(get(handles.showAllbutton,'Value')) %If the button is clicked
    for j=1:size(data{iteration}{1},1)
        line([data{iteration}{1}(j,1),data{iteration}{1}(j,3)],[data{iteration}{1}(j,2),data{iteration}{1}(j,4)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        hold on;
    end
    for k=1:size(data{iteration}{9},1)
        line([data{iteration}{9}(k,1),data{iteration}{9}(k,2)],[data{iteration}{10}(k,1),data{iteration}{10}(k,2)],'Color',[0 0 0],'LineStyle','--','LineWidth',1)
        hold on;
    end
    plot(data{iteration}{1}(:,1),data{iteration}{1}(:,2),'m*')
    hold on;
    plot(data{iteration}{4},data{iteration}{5},'mo','MarkerSize', 4.5);
    hold on;
else %Else clear the figure
    cla;
end
