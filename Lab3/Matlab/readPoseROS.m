function [x, y, a] = readPoseROS(sub)
    % Obtener el último mensaje
    poseMsg = sub.LatestMessage;
    
    % Accede a los datos del mensaje
    if ~isempty(poseMsg)
        x = poseMsg.X;
        y = poseMsg.Y;
        a = poseMsg.Theta;
        disp("x: " + x + " y: " + y + " theta: " + a);
    else
        x = nan;
        y = nan;
        a = nan;
        disp("x: " + x + " y: " + y + " theta: " + a);
    end
end