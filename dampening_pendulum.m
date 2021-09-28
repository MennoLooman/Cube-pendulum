clc
[pks,locs] = findpeaks(y(:,1),t);
[pks_min,locs_min] = findpeaks(-y(:,1),t);


plot(locs,pks,'*')
hold on
plot(locs_min,-pks_min,'*')

dt = (locs(end)-locs(1))/length(locs);

alpha = zeros(1,length(locs)-1);
for i=1:length(pks)-1
    alpha(i) = -1/dt * log(pks(i+1)/pks(i));
end

alpha_min = zeros(1,length(locs)-1);
for i=1:length(pks_min)-1
    alpha_min(i) = -1/dt * log(pks_min(i+1)/pks_min(i));
end

damp = mean(alpha);
damp_min = mean(alpha_min);

figure()
hold on
plot(alpha)
plot(alpha_min)

damp = (damp+damp_min)/2;