% Subject: AERO4500
% Task: Assignment 1
% Group 3
% Author: James Morgan

clear; clc; close all

%% Passenger and crew parameters

avg_person = 86;             % Average person mass (kg)
num_pax = 6;                 % Number of passengers (max 20)
num_crew = 2;                % Number of pilots

carry_on_weight = 7;         % Passenger carry-on baggage (kg)
check_in = 20;               % Passenger checked baggage (kg)
carry_on_weight_crew = 10;   % Crew baggage (kg)

% Crew and payload mass
W_crew = num_crew * (avg_person + carry_on_weight_crew);
W_payload = num_pax * (avg_person + carry_on_weight + check_in);

%% Hydrogen propulsion system

% All masses in kg

num_tanks = 5;
num_fuel_cells = 4;
num_inverters = 4;

W_tanks_full = 368;          % Five full hydrogen tanks
W_fuel_cells = 560;          % Four fuel cells
W_motor_inverters = 188+30;     % Motor and four inverters

% Total installed propulsion system including hydrogen
W_system = W_tanks_full + W_fuel_cells + W_motor_inverters;

% Hydrogen capacity of all five tanks
W_hydrogen = num_tanks * 28;

% Empty hydrogen tank mass
W_tanks_empty = W_tanks_full - W_hydrogen;

% Dry propulsion system mass
W_propulsion = W_tanks_empty + W_fuel_cells + W_motor_inverters;


%% Initial aircraft sizing

W_0_guess = 5000;            % Initial takeoff mass estimate (kg)

% Empty weight coeffs
A = 0.72;
C = -0.03;

% Empty weight fraction
empty_frac = @(W0) A * W0^C;

mass_balance = @(W0) W0 - (W_crew + W_payload + W_hydrogen + ...
    W_propulsion + empty_frac(W0) * W0);

% Iteratively solve for takeoff
W_0 = fzero(mass_balance, W_0_guess);


%% Final aircraft mass calculations

% Empty mass excluding separately added propulsion system
w_empty_frac = empty_frac(W_0);
W_empty = w_empty_frac * W_0;

% Hydrogen fuel fraction
fuel_frac = W_hydrogen / W_0;

% Propulsion system fraction
propulsion_frac = W_propulsion / W_0;

% Total operating empty mass
W_empty_total = W_empty + W_propulsion;

% Check mass balance
W_0_check = W_crew + W_payload + ...
    W_hydrogen + W_empty_total;


%% Display results

fprintf('\nAIRCRAFT MASS ESTIMATE\n');

fprintf('Crew mass:                 %.2f kg\n', W_crew);
fprintf('Payload mass:              %.2f kg\n', W_payload);

fprintf('\nHydrogen propulsion\n');

fprintf('Full hydrogen tanks:       %.2f kg\n', W_tanks_full);
fprintf('Hydrogen fuel mass:        %.2f kg\n', W_hydrogen);
fprintf('Empty tank mass:           %.2f kg\n', W_tanks_empty);
fprintf('Fuel cell mass:            %.2f kg\n', W_fuel_cells);
fprintf('Motor/inverter mass:       %.2f kg\n', W_motor_inverters);
fprintf('Dry propulsion mass:       %.2f kg\n', W_propulsion);

fprintf('\nAircraft sizing\n');

fprintf('Empty mass (excl. propulsion): %.2f kg\n', W_empty);
fprintf('Total operating empty mass:    %.2f kg\n', W_empty_total);

fprintf('Empty fraction (empirical):    %.4f\n', w_empty_frac);
fprintf('Hydrogen fuel fraction:        %.4f\n', fuel_frac);
fprintf('Propulsion mass fraction:      %.4f\n', propulsion_frac);

fprintf('\nCalculated takeoff mass:   %.2f kg\n', W_0);
fprintf('Mass balance check:        %.2f kg\n', W_0_check);


%% Wing lift calculations using XFOIL data

% Aircraft properties
g = 9.81;
W = W_0 * g;                 % Aircraft weight (N)

% Wing geometry
S = 50;                     % Wing area (m^2)
AR = 8;                     % Aspect ratio
b = sqrt(AR*S);             % Wingspan (m)
c_avg = S/b;                % Mean geometric chord (m)

%% Import XFOIL polar

filename = 'xf-n2415-il-1000000.csv';

% Skip daat at top of CSV
polar = readtable(filename, 'NumHeaderLines', 10);

alpha_data = polar.Alpha;
Cl_data = polar.Cl;
Cd_data = polar.Cd;

% Locate maximum section lift coefficient
[Cl_max_2D, idx_max] = max(Cl_data);
alpha_stall_2D = alpha_data(idx_max);

% Approximate conversion from 2D section to finite-wing CLmax
wing_CL_factor = 0.90;

CL_max_clean = wing_CL_factor * Cl_max_2D;

delta_CL_takeoff = 0.40;
delta_CL_landing = 0.70;

CL_max_takeoff = CL_max_clean + delta_CL_takeoff;
CL_max_landing = CL_max_clean + delta_CL_landing;

%% Atmospheric conditions

rho_SL = 1.2256;

% Cruise alt
h_cruise = 5000 / 3.281;

% ISA atmosphere
T0 = 288.15;
P0 = 101325;
lapse = 0.0065;
R = 287.05;

T_cruise = T0 - lapse*h_cruise;
P_cruise = P0 * (T_cruise/T0)^(g/(R*lapse));
rho_cruise = P_cruise/(R*T_cruise);

%% Aircraft aerodynamic parameters

e = 0.80;                   % Oswald efficiency
K = 1/(pi*e*AR);            % Induced drag factor

CD_other = 0.020;

% Flap drag increment for takeoff configuration
delta_CD_takeoff = 0.010;

%% Reynolds number checks

% Kinematic viscosity (m^2/s), approximate at sea level
nu_SL = 1.46e-5;

% Cruise true airspeed
V_cruise = 70;

% Approximate cruise kinematic viscosity
mu_cruise = 1.458e-6 * T_cruise^1.5 / ...
    (T_cruise + 110.4);

nu_cruise = mu_cruise/rho_cruise;

Re_cruise = V_cruise*c_avg/nu_cruise;

% XFOIL polar was generated at Re = 1 million
Re_XFOIL = 1e6;

%% Stall calcs

V_stall_clean = sqrt(2*W/(rho_SL*S*CL_max_clean));
V_stall_takeoff = sqrt(2*W/(rho_SL*S*CL_max_takeoff));
V_stall_landing = sqrt(2*W/(rho_SL*S*CL_max_landing));

%% Takeoff calcs

% Initial rotation speed
V_rotation = 1.2*V_stall_takeoff;
q_takeoff = 0.5*rho_SL*V_rotation^2;
CL_takeoff_required = W/(q_takeoff*S);
L_takeoff_max = q_takeoff*S*CL_max_takeoff;

%% Cruise calcs

q_cruise = 0.5*rho_cruise*V_cruise^2;

% Required lift in steady level flight
L_cruise = W;

% Required aircraft lift coefficient
CL_cruise = W/(q_cruise*S);

% Use positive angle, pre-stall XFOIL data for interpolation
valid = alpha_data >= -4 & alpha_data <= alpha_stall_2D;

Cl_valid = Cl_data(valid);
Cd_valid = Cd_data(valid);
alpha_valid = alpha_data(valid);

% Sort by lift coefficient
[Cl_valid, order] = sort(Cl_valid);

Cd_valid = Cd_valid(order);
alpha_valid = alpha_valid(order);

% Remove repeated lift coefficient values
[Cl_valid, unique_idx] = unique(Cl_valid);

Cd_valid = Cd_valid(unique_idx);
alpha_valid = alpha_valid(unique_idx);

% Check whether the required CL is in the polar range
if CL_cruise < min(Cl_valid) || CL_cruise > max(Cl_valid)
    error('Cruise CL is outside the selected XFOIL polar range.');

end

% Approximate section angle and profile drag at cruise CL
alpha_cruise_section = interp1(Cl_valid, alpha_valid, CL_cruise, 'linear');

CD_profile_cruise = interp1(Cl_valid, Cd_valid, CL_cruise, 'linear');

% Induced drag from finite-wing aspect ratio
CD_induced = K*CL_cruise^2;

% Total aircraft drag coefficient
CD_cruise = CD_profile_cruise + CD_induced + CD_other;

% Cruise drag force
D_cruise = q_cruise*S*CD_cruise;

% Aircraft lift-to-drag ratio
LD_cruise = CL_cruise/CD_cruise;

% Required propulsive power
P_propulsive = D_cruise*V_cruise;

% Propeller efficiency
eta_prop = 0.85;

% Required shaft power
P_shaft = P_propulsive/eta_prop;

%% Takeoff power requirement

% Runway parameters
s_TO = 400; % ground roll before rotation
mu_roll = 0.05; % conservative runway drag coefficent

% Propeller efficiency at liftoff
eta_prop_TO = 0.75;

% Liftoff speed
V_liftoff = V_rotation;

% Required constant acceleration
a_TO = V_liftoff^2/(2*s_TO);

CL_ground = 0.8;

% Use XFOIL profile drag at ground-roll CL
if CL_ground < min(Cl_valid) || CL_ground > max(Cl_valid)
    error('Ground-roll CL is outside XFOIL polar range.');

end

CD_profile_TO = interp1(Cl_valid, Cd_valid, CL_ground, 'linear');

% Total takeoff drag coefficient
CD_TO = CD_profile_TO + K*CL_ground^2 + CD_other + delta_CD_takeoff;

% Dynamic pressure at liftoff
q_TO = 0.5*rho_SL*V_liftoff^2;

% Lift at liftoff
L_TO = q_TO*S*CL_ground;

% Drag at liftoff
D_TO = q_TO*S*CD_TO;

% Normal reaction force
N_TO = max(W-L_TO,0);

% Rolling resistance
F_roll = mu_roll*N_TO;

% Required acceleration force
F_acceleration = W_0*a_TO;

% Total thrust required
T_TO = F_acceleration + D_TO + F_roll;

% Propulsive power at liftoff
P_propulsive_TO = T_TO*V_liftoff;

% Shaft power at liftoff
P_shaft_TO = P_propulsive_TO/eta_prop_TO;

%% Power comparison

P_max = max([P_shaft,P_shaft_TO]);

%% Display results

fprintf('\nXFOIL AIRFOIL DATA\n');

fprintf('Airfoil: NACA 2415\n');
fprintf('XFOIL Reynolds number: %.0f\n', Re_XFOIL);
fprintf('Estimated cruise Reynolds number: %.0f\n', Re_cruise);

fprintf('Maximum section Cl: %.4f\n',Cl_max_2D);
fprintf('Section stall alpha: %.2f deg\n',alpha_stall_2D);

fprintf('Estimated wing CLmax clean: %.4f\n',CL_max_clean);
fprintf('Estimated wing CLmax takeoff: %.4f\n',CL_max_takeoff);
fprintf('Estimated wing CLmax landing: %.4f\n',CL_max_landing);

%% Wing geometry

fprintf('\nWING GEOMETRY\n');

fprintf('Aircraft mass: %.2f kg\n',W_0);
fprintf('Aircraft weight: %.2f N\n',W);
fprintf('Wing area: %.2f m^2\n',S);
fprintf('Aspect ratio: %.2f\n',AR);
fprintf('Wingspan: %.2f m\n',b);
fprintf('Mean chord: %.2f m\n',c_avg);
fprintf('Wing loading: %.2f N/m^2\n',W/S);

%% Stall results

fprintf('\nSTALL CONDITIONS\n');
fprintf('Clean stall: %.2f m/s (%.2f kt)\n', V_stall_clean,V_stall_clean*1.94384);
fprintf('Takeoff stall: %.2f m/s (%.2f kt)\n', V_stall_takeoff,V_stall_takeoff*1.94384);
fprintf('Landing stall: %.2f m/s (%.2f kt)\n', V_stall_landing,V_stall_landing*1.94384);

%% Takeoff results

fprintf('\nTAKEOFF CONDITIONS\n');

fprintf('Rotation speed: %.2f m/s (%.2f kt)\n', V_rotation,V_rotation*1.94384);
fprintf('Required CL at rotation: %.4f\n', CL_takeoff_required);
fprintf('Max available lift: %.2f N\n',L_takeoff_max);
fprintf('Required acceleration: %.3f m/s^2\n',a_TO);
fprintf('Takeoff drag: %.2f N\n',D_TO);
fprintf('Required thrust: %.2f N\n',T_TO);

fprintf('Takeoff shaft power: %.2f kW\n', P_shaft_TO/1000);

%% Cruise results

fprintf('\nCRUISE CONDITIONS\n');

fprintf('Cruise altitude: %.0f m\n',h_cruise);
fprintf('Cruise density: %.4f kg/m^3\n',rho_cruise);
fprintf('Cruise speed: %.2f m/s (%.2f kt)\n', V_cruise,V_cruise*1.94384);
fprintf('Required aircraft CL: %.4f\n',CL_cruise);
fprintf('Approx section alpha: %.2f deg\n', alpha_cruise_section);

fprintf('XFOIL profile Cd: %.5f\n',CD_profile_cruise);
fprintf('Induced CD: %.5f\n',CD_induced);
fprintf('Other aircraft CD: %.5f\n',CD_other);

fprintf('Total cruise CD: %.5f\n',CD_cruise);
fprintf('Cruise drag: %.2f N\n',D_cruise);
fprintf('Cruise L/D: %.2f\n',LD_cruise);

fprintf('Cruise shaft power: %.2f kW\n', P_shaft/1000);

%% Power summary

fprintf('\nPOWER COMPARISON\n');
fprintf('Cruise shaft power: %.2f kW\n',P_shaft/1000);
fprintf('Takeoff shaft power: %.2f kW\n',P_shaft_TO/1000);

fprintf('Maximum evaluated power: %.2f kW\n', P_max/1000);

