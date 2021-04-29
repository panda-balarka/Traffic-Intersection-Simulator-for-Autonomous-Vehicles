function drawVehicle3D(VehicleList,CarWidth,printLabel)
height = 3;
for i = 1:length(VehicleList)
    PosX = VehicleList(i).position.x;
    PosY = VehicleList(i).position.y;
    PosZ = 20;
    phi = VehicleList(i).heading;
    color = VehicleList(i).color;
    recX0=[-VehicleList(i).length/2 VehicleList(i).length/2 VehicleList(i).length/2 -VehicleList(i).length/2 -VehicleList(i).length/2];
    recY0=[-CarWidth/2 -CarWidth/2 CarWidth/2 CarWidth/2 -CarWidth/2];
    RrecX0=recX0*cos(phi)-recY0*sin(phi);
    RrecY0=recX0*sin(phi)+recY0*cos(phi);
    recX=RrecX0 + PosX;
    recY=RrecY0 + PosY;
    
    drawBox(recX,recY,height,color)
    if printLabel == 1
        text(PosX,PosY,PosZ,num2str(VehicleList(i).ID))
    end
    if printLabel == 2
        text(PosX,PosY,PosZ,num2str(VehicleList(i).speed,4))
    end
end