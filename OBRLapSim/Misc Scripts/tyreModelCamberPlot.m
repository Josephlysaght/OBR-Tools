testCarInitialization

SA = linspace(-25,25,100);
Fz =1000;
SR = 0.00;
IA = linspace(0,4,5);
Fy = zeros(length(SA),length(IA));

for i = 1:length(IA)
    for j = 1:length(SA)
    Fy(j,i) = LTSFyCombinedMF52(Car,SA(j),SR,Fz,IA(i));
    %Fy(j,i) = LTSFyPureMF52(Car,SA(j),Fz,IA(i));
    end
end

legend_text=cell(length(IA),1);
for p=1:length(IA)
    legend_text{p}=strcat(num2str(IA(p)),'deg');
end

cm=colormap(hsv(length(IA))); %make color map for plotting
figure
hold on
for i = 1:length(IA)
    plot(SA,Fy(:,i),'Color',cm(i,:))
end
legend(legend_text);
xlabel('Slip Angle (deg)')
ylabel('Lateral Force (N)')