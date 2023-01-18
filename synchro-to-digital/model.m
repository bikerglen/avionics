# setup
length=25e3/40e3;  # time in seconds
Fs=40e3;           # sample rate
T=1/Fs;            # period

# create time vector
t=0:T:length;

# create reference waveform
ref = sin (2*pi*400*t);

# create set points
setpts = zeros (size (t));
setpts(    1:5000) = -90;
setpts( 5001:10000) = -90:90/5000:-90/5000;
setpts(10001:15000) = 0;
setpts(15001:20000) = 0:90/5000:90-90/5000;
setpts(20001:25000) = +90;
setpts(25001) = +90;
setpts = setpts * pi/180;

# create synchro waveforms
s1ms3 = ref .* sin (setpts);
s3ms2 = ref .* sin (setpts + 120*pi/180);
s2ms1 = ref .* sin (setpts + 240*pi/180);

# create sin and cos inputs using scott transformation
sinin = s1ms3;
cosin = 2/sqrt(3) * (s3ms2 + 0.5 * s1ms3);

# convert ref waveform into square waveform
refsq = sign(ref);

# init state variables
theta = zeros(size(t));
delta = zeros(size(t));
demod = zeros(size(t));

for i=1:size(t)(2)-1
  # calculate AC error term
  delta(i) = sinin(i)*cos(theta(i)) - cosin(i)*sin(theta(i));

  # demodulate AC error term
  demod(i) = refsq(i) * delta(i);

  # apply gain term to demodulated error term and integrate
  newtheta = theta(i) + 1/64*demod(i);

  # wrap from -pi to +pi
  theta(i+1) = mod(newtheta+pi, 2*pi)-pi;
endfor

### FIGURE 1 ###

figure (1, 'position',[0,0,1280,600]);
plot (t, ref);
title('Reference Voltage');
axis ([0 length -1.5 1.5])
xlabel ('time (seconds)');
ylabel ('voltage (volts)');
legend ('VR2-VR1');
print -dpng figure1.png

### FIGURE 2 ###

figure (2, 'position',[0,0,1280,800]);

subplot(3,1,1);
plot (t, setpts);
title('Input Shaft Angle');
axis ([0 length])
xlabel ('time (seconds)');
ylabel ('shaft angle (radians)');
legend ('Shaft Angle');

subplot(3,1,2);
plot (t, s1ms3, t, s3ms2, t, s2ms1);
title('Synchro Voltages');
axis ([0 length -1.5 1.5])
xlabel ('time (seconds)');
ylabel ('voltage (volts)');
legend ('VS1-VS3','VS3-VS2','VS2-VS1');

subplot(3,1,3);
plot (t, sinin, t, cosin);
title('Sine / Cosine Voltages');
axis ([0 length -1.5 1.5])
xlabel ('time (seconds)');
ylabel ('voltage (volts)');
legend ('Vsin', 'Vcos');

print -dpng figure2.png

### FIGURE 3 ###

figure (3, 'position',[0,0,1280,600]);
y1 = [delta;demod];
y2 = [setpts;theta];
ax = plotyy (t, y1, t, y2);
title('Error, Demodulated Error, Output Angle');
legend ('Error', 'Demodulated Error', 'Input Shaft Angle', 'Output Angle');
axis ([0 length -0.5 +0.5 -2 +2])
xlabel ('time (seconds)');
ylabel (ax(1), "errors (volts)");
ylabel (ax(2), "output angle (radians)");
print -dpng figure3.png

### FIGURE 4 ###

figure (4, 'position',[0,0,1280,600]);
y1 = [delta;demod];
y2 = [setpts;theta];
ax = plotyy (t, y1, t, y2);
title('Error, Demodulated Error, Output Angle');
legend ('Error', 'Demodulated Error', 'Input Shaft Angle', 'Output Angle');
axis (ax(1), [0.1 0.275 -0.2 +0.2])
axis (ax(2), [0.1 0.275 -2 +2])
xlabel ('time (seconds)');
ylabel (ax(1), "errors (volts)");
ylabel (ax(2), "output angle (radians)");
print -dpng figure4.png

### FIGURE 5 ###

figure (5, 'position',[0,0,1280,600]);
plot (t, setpts, t, theta);
title('Input Shaft Angle, Output Angle');
axis ([0 length])
xlabel ('time (seconds)');
ylabel ('shaft angle (radians)');
legend ('input angle', 'output angle');
print -dpng figure5.png

