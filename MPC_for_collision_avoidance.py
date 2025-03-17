import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 25
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = 4
        self.y_obs = 0.1

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]
        
        x_t_1 = x_t+ (v_t*np.cos(psi_t)*dt);
        y_t_1 = y_t+ (v_t*np.sin(psi_t)*dt);
        psi_t_1 = psi_t + (v_t/2.5)*np.tan(steering)*dt;
        v_t_1 = v_t+ pedal*dt - (v_t/25);

        return [x_t_1, y_t_1, psi_t_1, v_t_1]


    def cost_function(self,u, *args):
        
        state = args[0]
        ref = args[1]
        x_g = ref[0];
        y_g = ref[1];
        psi_g = ref[2];
        cost=0;
        
        for i in range(0,self.horizon):
            v_prev = state[3];
            state = self.plant_model(state,self.dt,u[2*i],u[2*i+1])
            x_n = state[0];
            y_n = state[1];
            psi_n = state[2];
            v_n = state[3];
            
            #goal distance cost
            cost += abs(x_g-x_n) + abs(y_g-y_n);
            
            
            #yaw angle cost
            cost += abs(psi_g-psi_n)**2;
            
            #acceleration cost
            #a_n = (v_n - v_prev)/self.dt;    
            #if a_n >1.5:
                #cost +=200;
            
            #obstacle cost
            
            dis = np.sqrt((x_n-self.x_obs)**2 + (y_n- self.y_obs)**2)
            
            if dis > 2:
                cost += 5;
            else:
                
                cost += 5/dis;
            
                
        return cost
sim_run(options, ModelPredictiveControl)
