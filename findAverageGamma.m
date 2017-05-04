function avg = findAverageGamma(num)
    t = zeros(num,1); 
    for i = 1:num
      t(i) = ComplementaryFilter(simulatedData(5,1),1);
    end
    avg = mean(t);
end
