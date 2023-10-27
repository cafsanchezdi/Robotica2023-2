function [] = cleanTurtle()
    clear;
    clc;

    % Reiniciar Turtle
    service_name = '/reset';
    service_type = 'std_srvs/Empty';
    client = rossvcclient(service_name, service_type);
    req = rosmessage(service_type);
    call(client, req);
end