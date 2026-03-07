clear; clc; close all

%% Sheet parameters
shear = 1500;
vertical_h = @(x) 0.100 - x/15; % length in m as input, 0 to 750mm
skin_length = 250e-3;
skin_thickness = 0.4064e-3;
spar_thickness = 0.8128e-3;
spar_thickness = 2.54e-3;



%% Known data (from table) 6061 T4
tensile_yield_ksi = 16;         % yield strength in ksi
tensile_ultimate_ksi = 30;         % yield strength in ksi
compressive_yield_ksi = 16;         % yield strength in ksi
shear_ultimate_ksi = 20;

tensile_yield = tensile_yield_ksi * 6.895e6;
tensile_ultimate = tensile_ultimate_ksi * 6.895e6;
compressive_yield = compressive_yield_ksi * 6.895e6;
shear_ultimate = shear_ultimate_ksi * 6.895e6;
E = 9.9e3 * 6.895e6;

%% Bracket parameters
bracket_t1 = 0.4064e-3;   % thickness
bracket_h1 = 12.5e-3;     % leg length

%% calcs

% bending moment at section x
bending_moment = @(x) (0.75 - x) * shear; % length in m as input, 0 to 750mm

% Spar bracket area
bracket_A = 2*(bracket_h1 * bracket_t1) - bracket_t1^2;

% Tamper angle
tamper_angle = atan(50/750); % rads

% Idealised boom area
B = @(x) bracket_A + (skin_length * skin_thickness/6 * (2+1)) + ...
    (vertical_h(x) * spar_thickness/6 * (2-1));

% Moment of inerta about the bending axis
I = @(x) (B(x)+B(x)+B(x)+B(x)) .* (vertical_h(x)/2).^2;

% Boom stress
boom_stress = @(x) bending_moment(x) .* vertical_h(x) .* 0.5 ./ I(x);

% Boom stress at the underside of the skin
boom_stress_under = @(x) boom_stress(x) ./ cos(tamper_angle);

% correct up to here

% Boom load
Pz = @(x) boom_stress(x) .* B(x);

% Correctin for tamper angle
Py = @(x) Pz(x) .* tan(tamper_angle);

% correct shear for 2 tampered booms
shear_corrected = @(x) shear - 2*Py(x);

% Shear flow calculations
qb12_init = @(x) 0;
qb23_init = @(x) qb12_init(x) - (shear_corrected(x) ./ I(x)).*B(x).*(vertical_h(x)/2);
qb34_init = @(x) qb23_init(x) - (shear_corrected(x) ./ I(x)).*B(x).*(-vertical_h(x)/2);
qb41_init = @(x) qb34_init(x) - (shear_corrected(x) ./ I(x)).*B(x).*(-vertical_h(x)/2);

% Constant shear flow
Qs0 = @(x) ((-shear_corrected(x)*skin_length/2) ...
            - qb23_init(x).*(vertical_h(x))*skin_length/2 ...
            - qb41_init(x).*(vertical_h(x))*skin_length/2) ...
            ./ (2*vertical_h(x)*skin_length);

% summation of constant and dynamic shear flows
qb12 = @(x) qb12_init(x) + Qs0(x);
qb23 = @(x) qb23_init(x) + Qs0(x);
qb34 = @(x) qb34_init(x) + Qs0(x);
qb41 = @(x) qb41_init(x) + Qs0(x);

% Conversion of shear flows to shear stresses in each panel
shear_12 = @(x) qb12(x)/skin_thickness;
shear_23 = @(x) qb23(x)/spar_thickness;
shear_34 = @(x) qb34(x)/skin_thickness;
shear_41 = @(x) qb41(x)/spar_thickness;


% define the input x range for plotting
input = 0:0.01:0.75;
shear_top_skin = shear_12(input);
shear_bottom_skin = shear_23(input);
shear_front_spar = shear_34(input);
shear_aft_spar = shear_41(input);

crit_top_skin = max(abs(shear_top_skin));
crit_bottom_skin = max(abs(shear_bottom_skin));
crit_front_skin = max(abs(shear_front_spar));
crit_aft_skin = max(abs(shear_aft_spar));

MOS_top_skin = s_margin(crit_top_skin,shear_ultimate)
MOS_bottom_skin = s_margin(crit_bottom_skin,shear_ultimate)
MOS_front_spar = s_margin(crit_front_skin,shear_ultimate)
MOS_aft_spar = s_margin(crit_aft_skin,shear_ultimate)

stress_spars = boom_stress_under(input);
crit_spar = max(abs(stress_spars));

MOS_spar_ultimate = s_margin(crit_spar,tensile_ultimate)
MOS_spar_yield = s_margin(crit_spar,tensile_yield)


%figure(1)
%subplot(5,1,1)
%plot(input*1000,vertical_h(input)*1000)
%grid on
%xlabel('Length (mm)')
%ylabel('height (mm)')
%title('Length to height')
%
%
%subplot(5,1,2)
%plot(input,bending_moment(input))
%grid on
%xlabel('Length (mm)')
%ylabel('Bending moment (N/m)')
%title('Bending moment vs length')
%
%subplot(5,1,3)
%plot(input,B(input))
%grid on
%xlabel('Length (mm)')
%ylabel('Boom area (m^2)')
%title('Boom area vs length')
%
%subplot(5,1,4)
%plot(input,boom_stress(input)/1e6)
%xlabel('Length (mm)')
%ylabel('Boom stress (MPa)')
%title('Bending stress in top booms')
%grid on
%
%subplot(5,1,5)
%plot(input,boom_stress_under(input)/1e6)
%xlabel('Length (mm)')
%ylabel('Boom stress (MPa)')
%title('Bending stress in bottom booms')
%grid on
%%
figure(2)
subplot(4,1,1)
plot(input*1000,shear_top_skin/1e6)
grid on
xlabel('Length (mm)')
ylabel('stress (MPa)')
title('Top skin shear stress vs length')
legend(sprintf('Max = %.2f MPa', crit_top_skin/1e6))


subplot(4,1,2)
plot(input,shear_front_spar/1e6)
grid on
xlabel('Length (mm)')
ylabel('stress (MPa)')
title('Front spar shear stress vs length')
legend(sprintf('Max = %.2f MPa', crit_front_skin/1e6))

subplot(4,1,3)
plot(input,shear_bottom_skin/1e6)
grid on
xlabel('Length (mm)')
ylabel('stress (MPa)')
title('Bottom skin shear stress vs length')
legend(sprintf('Max = %.2f MPa', crit_bottom_skin/1e6))

subplot(4,1,4)
plot(input,shear_aft_spar/1e6)
xlabel('Length (mm)')
ylabel('stress (MPa)')
title('Rear spar shear stress vs length')
legend(sprintf('Max = %.2f MPa', crit_aft_skin/1e6))
grid on

spar_stress_MPa = stress_spars/1e6;
yield_allow = compressive_yield/1e6;
ultimate_allow = shear_ultimate/1e6;
yield_idx = find(abs(spar_stress_MPa) >= yield_allow,1);
ult_idx = find(abs(spar_stress_MPa) >= ultimate_allow,1);

figure(3)
subplot(1,1,1)
plot(input,stress_spars/1e6)
xlabel('Length (mm)')
ylabel('stress (MPa)')
title('Spar stress vs length')
legend(sprintf('Max = %.2f MPa', crit_spar/1e6))
grid on




function MoS = s_margin(design, allowed)

    MoS = (allowed / design) -1;

end