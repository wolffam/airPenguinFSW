fclose(instrfind);

comport = serial('/dev/cu.usbserial-DN00Q97Q', 'BaudRate', 9600);  % xbee
fopen(comport);

i = 1;

while exist(strcat('RocketDataPressure',num2str(i),'.txt'), 'file')==2
    i = i+1;
end

fileID1 = fopen(strcat('RocketDataPressure',num2str(i),'.txt'), 'w');
%fprintf(fileID1, 'State       Latitude             Longitude             Altitude             GPS Altitude          X Acceleration         Y Acceleration        Z Acceleration             PFOX              PCC             Temperature          Time \n');
fprintf(fileID1, 'State       Latitude             Longitude             GPS Altitude              PFOX              PCC                   Time \n');
fclose(fileID1);

tic();

while(1)
   if comport.BytesAvailable > 0
       
       clc;
       
       Time = toc();
       State = fscanf(comport, '%f');
       Latitude = fscanf(comport, '%f');
       Longitude = fscanf(comport, '%f');
       %Altitude = fscanf(comport, '%f');
       GPS_Altitude = fscanf(comport, '%f');
       %Linear_Acceleration_x = fscanf(comport, '%f');
       %Linear_Acceleration_y = fscanf(comport, '%f');
       %Linear_Acceleration_z = fscanf(comport, '%f');
       PFOX = fscanf(comport, '%f');
       PCC = fscanf(comport, '%f');
       %Temperature = fscanf(comport, '%f');
       
       disp(strcat('Time: ',num2str(Time)));
       disp(strcat('State: ',num2str(State)));
       disp(strcat('Latitude: ',num2str(Latitude)));
       disp(strcat('Longitude: ',num2str(Longitude)));
       %disp(strcat('Altitude: ',num2str(Altitude)));
       disp(strcat('GPS Altitude: ',num2str(GPS_Altitude)));
       %disp(strcat('X Acc: ',num2str(Linear_Acceleration_x)));
       %disp(strcat('Y Acc: ',num2str(Linear_Acceleration_y)));
       %disp(strcat('Z Acc: ',num2str(Linear_Acceleration_z)));
       disp(strcat('PFOX: ',num2str(PFOX)));
       disp(strcat('PCC: ',num2str(PCC)));
       %disp(strcat('Temperature: ',num2str(Temperature)));
       disp('Press Ctrl-C to Exit');

  
    fileID1 = fopen(strcat('RocketDataPressure',num2str(i),'.txt'), 'a');
    %fprintf(fileID1,'%d          %d          %d          %d          %d          %d          %d          %d          %d          %d          %d          %d \n', [State; Latitude; Longitude; Altitude; GPS_Altitude; Linear_Acceleration_x; Linear_Acceleration_y; Linear_Acceleration_z; PFOX; PCC; Temperature; Time]);
    fprintf(fileID1,'%d          %d          %d          %d          %d          %d          %d         \n', [State; Latitude; Longitude; GPS_Altitude; PFOX; PCC; Time]);
    fclose(fileID1);
   end 
   


end