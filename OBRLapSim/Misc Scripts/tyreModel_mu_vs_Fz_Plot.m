testCarInitialization

SA = linspace(-25,25,100);
Fz =[200,300,400,500,600,700,800,900,1000,1100,1200];
SR = 0.00;
IA = 0;
Fy = zeros(length(SA),length(IA));

for i = 1:length(Fz)
    for j = 1:length(SA)
    Fy(j,i) = LTSFyCombinedMF52(Car,SA(j),SR,Fz(i),IA);
    %Fy(j,i) = LTSFyPureMF52(Car,SA(j),Fz,IA(i));
    end
end

legend_text=cell(length(Fz),1);
for p=1:length(Fz)
    legend_text{p}=strcat(num2str(Fz(p)),'N');
end

cm=colormap(jet(length(Fz))); %make color map for plotting
for i = 1:length(Fz)
    plot(SA,Fy(:,i)/Fz(i),'Color',cm(i,:))
    hold on
end
legend(legend_text);



for i = 1:length(Fz)
    mu(:,i) = Fy(:,i)./Fz(i);
end
figure
plot(Fz,max(mu))
xlabel('Fz (N)')
ylabel('Friction Coefficeint')
title('Tyre Load Sensitivity at 0 Inclination Angle')
