function qnew = steer(qnearest, qrand, numNodes)
%returns a configuration resulting from steering from qnearest towards
%qrand
    qdir = qrand - qnearest;
    delta_q = 0.4*(log(numNodes)/numNodes)^(1/4); %vary length of branch as tree grows
    if norm(qdir) > delta_q
        qdir = delta_q* qdir/norm(qdir);
    end
    qnew = qnearest + qdir;
end
