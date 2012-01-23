% Written by Shuhao Wu, Team 4069
% This code is licensed under GNU GPLv3

% Where downwards and backwards is negative!

1;

function main()

	% Modify these constants if necessary
	M = 0.317514659; % Mass of the 2012 basketball in kg. (Official number)
	R = 0.635 / 2 / pi; % Radius of the 2012 basketball in m. (Official number)
	STEP = 0.001; % STEP size in seconds for simulation.
	MAX_TIME = 10; % maximum number of seconds to simulate or hit floor.

	INITIAL_V = 10; % initial STEP size in m/s
	THETA = 45; % Firing angle in degrees
	INITIAL_HEIGHT = 1.5; % Initial height in meters.

	DRAG_COEFFICIENT = 0.15; % For drag
	LIFT_COEFFICIENT = 0.2; % For magnus effect
	AIR_DENSITY = 1.178; % Air density in kg/m^3
	% Constant area ends

	GRAVFORCE = [0; -9.81*M];
	THETA *= pi / 180;
	standard_location = [0; INITIAL_HEIGHT];
	drag_location = [0; INITIAL_HEIGHT];
	magnus_location = [0; INITIAL_HEIGHT];

	standard_v = [INITIAL_V * cos(THETA); INITIAL_V * sin(THETA)];
	drag_v = standard_v;
	magnus_v = standard_v;

	standard_x = [];
	standard_y = [];

	drag_x = [];
	drag_y = [];

	magnus_x = [];
	magnus_y = [];

	standard_done = 0;
	drag_done = 0;
	magnus_done = 0;
	for i=STEP:STEP:MAX_TIME
		if sum([standard_done drag_done magnus_done]) == 3
			break
		end
		if standard_done == 0
			[standard_location standard_v] = newPos(standard_location, standard_v, R, M, 0, 0, 0, STEP, GRAVFORCE);
			if standard_location(2) < 0
				standard_done = 1;
			else
				standard_x(floor(i/STEP)) = standard_location(1);
				standard_y(floor(i/STEP)) = standard_location(2);
			end
		end

		if drag_done == 0
			[drag_location, drag_v] = newPos(drag_location, drag_v, R, M, AIR_DENSITY, DRAG_COEFFICIENT, 0, STEP, GRAVFORCE);
			if drag_location(2) < 0
				drag_done = 1;
			else
				drag_x(floor(i/STEP)) = drag_location(1);
				drag_y(floor(i/STEP)) = drag_location(2);
			end
		end

		if magnus_done == 0
			[magnus_location, magnus_v] = newPos(magnus_location, magnus_v, R, M, AIR_DENSITY, DRAG_COEFFICIENT, LIFT_COEFFICIENT, STEP, GRAVFORCE);
			if magnus_location(2) < 0
				magnus_done = 1;
			else
				magnus_x(floor(i/STEP)) = magnus_location(1);
				magnus_y(floor(i/STEP)) = magnus_location(2);
			end
		end
	end


	plot(standard_x, standard_y, 'ro', 'MarkerSize', 4);
	hold on;

	plot(drag_x, drag_y, 'bo', 'MarkerSize', 4);
	hold on;

	plot(magnus_x, magnus_y, 'go', 'MarkerSize', 4);

	xlabel("X Distance (m)");
	ylabel("Y Distance (m)");
	title(sprintf("Firing angle: %f degrees | Initial V: %f m/s", THETA*180/pi, INITIAL_V));
	legend("no drag/magnus", "drag, no magnus", "drag, magnus");
end

function [location v] = newPos(location, v, r, m, air_density, drag_coefficient, lift_coefficient, step, gravforce)
	location += v .* step;
	% Done in 3 lines for readability
	F = dragforce(v, r, air_density, drag_coefficient);
	F += magnusforce(v, r, air_density, lift_coefficient);
	F += gravforce;
	v = ((v .* step) + (acc(F, m) .* (step^2))) ./ step;
end

function a = acc(F, m)
	a = F./m;
end

function F = dragforce(v, r, air_density, drag_coefficient)
	% Even though drag_coefficient depends on v, let's just estimate it.
	% v is a vector of vx and vy;
	% in efficient. Optimization will be extracting the temp variable outside the
	% mainloop and not compute it all the time
	temp = -0.5 * drag_coefficient * pi * r^2 * air_density;
	F = temp .* (v.^2);
end

function F = magnusforce(v, r, air_density, lift_coefficient)
	% Assume lift force. So backspin..
	F = [0.5 * air_density * pi * r^2 * lift_coefficient * v(1)^2; 0];
end


main();
