/*******************************/
/* Control and Simulation Code */
/*******************************/

// PID controller
class PID {
  constructor(Kp, Ki, Kd, set_point) {
    this.Kp = Kp;
    this.Ki = Ki;
    this.Kd = Kd;
    this.set_point = set_point;
    this.int_term = 0
    this.derivative_term = 0
    this.last_error = null
  }

  get_control(measurement, dt){
     var error = this.set_point - measurement;
     this.int_term += error*this.Ki*dt;
     if (this.last_error != null){
       this.derivative_term = (error-this.last_error)/dt*this.Kd
     }
     
     this.last_error = error
     return this.Kp * error + this.int_term + this.derivative_term
  }       
}

// temperature simulation
const alpha = 1;
const beta = 40;
const T_ambient =20;
const T_desired = 37;
const T_start = 25;


function next_temp(u, T, dt){
    return T+alpha*(T_ambient-T)*dt + beta * u *dt
}

function clip(x, xmin, xmax) // simple helper
{
  if (x<xmin)
    return xmin;
  if (x>xmax)
    return xmax;
  return x;
}

function run_temp_sim(controller)
{

  var dt = 0.1; // Every time interval dt we set a new control value
  var T = T_start;
  var T_list = [T];
  var time_list = [0];
  
  const num_steps = 20;
  
  for (let k = 0; k < num_steps; k++) {
    // ask controller for u value
    var u = controller.get_control(T,dt);
    // device only allows to set u between 0 and 1:
    u = clip(u, 0, 1);
    // simulate what the temperature will be after time interval dt
    T = next_temp(u, T, dt);
    T_list.push(T);
    time_list.push((k+1)*dt);
  }
  return [time_list, T_list];
}

/*******************************/
/* Visualization code **********/
/*******************************/

let states; // 2d array containing 1d time array and 1d temperature array
let controller;

let slider_p;
let slider_i;
let slider_d;

// layout for plotly plot
let layout = {
  title: '',
  xaxis: {
    title: 'Time',
  },
  yaxis: {
    title: 'Temperature',
    showline: false,
    range: [19, 42]
  }
};


function re_run_sim() {
  controller = new PID(slider_p.value()/10000.0,
                       slider_i.value()/10000.0,
                       slider_d.value()/10000.0,
                       T_desired);
  states = run_temp_sim(controller);
}


function setup(){

  super_group = createDiv('');
  
  super_group.style('width', '100%')
  
  group_p = createDiv('');
  group_p.parent(super_group);
  slider_p = createSlider(0,10000, 0);
  slider_p.style('width', '50%');
  slider_p.input(re_run_sim);
  slider_p.parent(group_p);
  slider_p_text = createSpan();
  slider_p_text.parent(group_p);
  
  group_i = createDiv('');
  group_i.parent(super_group);
  slider_i = createSlider(0,2000, 0);
  slider_i.style('width', '50%');
  slider_i.input(re_run_sim);
  slider_i.parent(group_i);
  slider_i_text = createSpan();
  slider_i_text.parent(group_i);
  
  group_d = createDiv('');
  group_d.parent(super_group);
  slider_d = createSlider(-30,30, 0);  
  slider_d.style('width', '50%');
  slider_d.input(re_run_sim);
  slider_d.parent(group_d);
  slider_d_text = createSpan();
  slider_d_text.parent(group_d);
  
  controller = new PID(0,0,0,T_desired);
  states = run_temp_sim(controller);  

}

function my_display_float(x,n) { // simple helper
  return Number.parseFloat(x).toFixed(n);
}


function draw() {    
  var trace_T = {
    x: states[0],
    y: states[1],
    name : 'T',
    type: 'line'
  };
  
  var trace_T_desired = {
    x: states[0],
    y: Array(states[0].length).fill(T_desired),
    name : 'T_desired',
    type: 'line'
  };
  
  super_group.position(0.25*windowWidth,0);

  slider_p_text.html('  Kp = '+my_display_float(slider_p.value()/10000.0,2)+ ' ');
  slider_i_text.html('  Ki = '+my_display_float(slider_i.value()/10000.0,2)+ ' ');
  slider_d_text.html('  Kd = '+my_display_float(slider_d.value()/10000.0,4));
    
  p = Plotly.newPlot('myDiv', [trace_T_desired,trace_T], layout,{displayModeBar: false});

}
