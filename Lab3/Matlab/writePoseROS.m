function [] = writePoseROS(pub,x,y,a)
    % Crear Mensaje
    msg = rosmessage(pub);

    % Editar Mensaje
    if ~isnan(x)
        msg.Linear.X = x;
    end
    if ~isnan(y)
        msg.Linear.Y = y;
    end
    if ~isnan(a)
        msg.Angular.Z = a;
    end

    % Enviar Mensaje
    send(pub,msg);
end