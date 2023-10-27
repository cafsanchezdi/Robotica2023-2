function [sub] = subscribeROS(topicName,messageType)    
    % Crea el suscriptor
    sub = rossubscriber(topicName, messageType);
end