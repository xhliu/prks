%% links(j) and links(k) contend given ER and signal map in a given slot?
% @param ers:           [node, neighbor, is_sender, er_thres, slot]
% @param link_gains:    [link, in_gain]
function y = isContend(links, j, k, slot, ers, link_gains)
    TX_POWER = -25;

    % conservative
    y = true;
    
    % for simplicity
    s1 = links(j, 1);
    r1 = links(j, 2);
    s2 = links(k, 1);
    r2 = links(k, 2);
    
    % 1) share nodes
    if (s1 == s2 || s1 == r2 || r1 == s2 || r1 == r2)
       return;
    end

    e = ers;
%     e = e(e(:, 5) == slot, :);
    if isempty(e)
        return;
    end
    
    g = link_gains;

    % 2) in each other's ER
    % s1
    thres = e(e(:, 1) == s1 & e(:, 2) == r1 & e(:, 3) == 1, 4);
    if isempty(thres)
        return;
    end
    g1 = g(g(:, 1) == s1, :);
    % s2 -> s1
    gain = g1(g1(:, 2) == s2, 3);
    if isempty(gain)
        return;
    end
    if (TX_POWER - gain(1)) >= thres(1)
        return;
    end
    % r2 -> s1
    gain = g1(g1(:, 2) == r2, 3);
    if isempty(gain)
        return;
    end
    if (TX_POWER - gain(1)) >= thres(1)
        return;
    end

    
    % r1
    thres = e(e(:, 1) == r1 & e(:, 2) == s1 & e(:, 3) == 0, 4);
    if isempty(thres)
        return;
    end
    g1 = g(g(:, 1) == r1, :);
    % s2 -> r1
    gain = g1(g1(:, 2) == s2, 3);
    if isempty(gain)
        return;
    end
    if (TX_POWER - gain(1)) >= thres(1)
        return;
    end
    % r2 -> r1
    gain = g1(g1(:, 2) == r2, 3);
    if isempty(gain)
        return;
    end
    if (TX_POWER - gain(1)) >= thres(1)
        return;
    end
    
    
    % s2
    thres = e(e(:, 1) == s2 & e(:, 2) == r2 & e(:, 3) == 1, 4);
    if isempty(thres)
        return;
    end
    g1 = g(g(:, 1) == s2, :);
    % s1 -> s2
    gain = g1(g1(:, 2) == s1, 3);
    if isempty(gain)
        return;
    end
    if (TX_POWER - gain(1)) >= thres(1)
        return;
    end
    % r1 -> s2
    gain = g1(g1(:, 2) == r1, 3);
    if isempty(gain)
        return;
    end
    if (TX_POWER - gain(1)) >= thres(1)
        return;
    end
           
    
    % r2
    thres = e(e(:, 1) == r2 & e(:, 2) == s2 & e(:, 3) == 0, 4);
    if isempty(thres)
        return;
    end
    g1 = g(g(:, 1) == r2, :);
    % s1 -> r2
    gain = g1(g1(:, 2) == s1, 3);
    if isempty(gain)
        return;
    end
    if (TX_POWER - gain(1)) >= thres(1)
        return;
    end
    % r1 -> r2
    gain = g1(g1(:, 2) == r1, 3);
    if isempty(gain)
        return;
    end
    if (TX_POWER - gain(1)) >= thres(1)
        return;
    end
    
    y = false;
end