function [] = teleportPoseROS(x,y,a)
    service_name = '/turtle1/teleport_absolute';
    service_type = 'turtlesim/TeleportAbsolute';
    
    % Crear un cliente de servicio
    client = rossvcclient(service_name, service_type);
    
    % Crear un mensaje de solicitud
    msgServ = rosmessage(service_type);
    
    % Establecer la posición y orientación a la que deseas teleportar
    msgServ.X = x; 
    msgServ.Y = y;   
    msgServ.Theta = a;
    
    % Llamar al servicio para teleportar
    call(client, msgServ);
end