#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle) {
    this->p_error = 0;
    this->i_error = 0;
    this->d_error = 0;

    this->last_cte = 0;

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    this->dp = {.1 * Kp, .1 * Ki, .1 * Kd};
    this->thresh = 1e-5;
    this->p_idx = 0;
    this->step = 1;
    this->sim_period = 2000;
    this->n_settle = 500;
    this->twiddle = twiddle;

    this->add_step = false;
    this->sub_step = false;

    this->error_sum = 0;
    this->error_min = 1e10;

    this->n_runs = 0;
}

void PID::UpdateError(double cte) {

    /*
    * PID part
    */

    this->p_error = cte;
    this->i_error += cte;
    // not fully corrected since this should be the time derivative but division by delta_t will be considered in Kd
    this->d_error = cte - last_cte;

    this->last_cte = cte;

    double sum_deltas = 0; 
    for (unsigned int i = 0; i < dp.size(); i++) sum_deltas += dp[i];

    /*
    * Twiddle part
    */
    if (twiddle && sum_deltas>thresh){
        // only take error after settling time or else first error will be smallest due to low speed
        if (step % (sim_period + n_settle) > n_settle) {
            this->error_sum += cte*cte;
        }
        // simulation exceeds sim period -> parameter adjustment
        if (step % (sim_period+n_settle)==0) {
            cout<<"Step "<<step<<" Sum Error: "<< error_sum<<" Best Error: "<<error_min<<endl;
            // check if parameterchange brought improvement
            if (error_sum < error_min) {
                this->error_min = error_sum;
                if (n_runs != 0) {
                    this->dp[p_idx] *= 1.1;
                }
                this->p_idx = (p_idx+1) % 3;
                this->add_step = this->sub_step = false;
                cout<<"current delta vector: "<<dp[0]<<" "<<dp[1]<<" "<<dp[2]<<endl;
            }
            //
            if (!add_step && !sub_step) {
                AdjustParam(p_idx, dp[p_idx]);
                this->add_step = true;
            } else if (add_step && !sub_step) {
                AdjustParam(p_idx, -2*dp[p_idx]);
                this->sub_step = true;
            } else {
                AdjustParam(p_idx, dp[p_idx]);
                this->dp[p_idx] *= 0.9;
                this->p_idx = (p_idx+1) % 3;
                this->add_step = this->sub_step = false;
            }
            this->error_sum = 0;
            cout<<"New Parameters, Kp: "<<Kp<<" Ki: "<<Ki<<" Kd: "<<Kd<<endl;
            this->n_runs += 1;
        }
        this->step += 1;
    }
}

double PID::TotalError() {
}

void PID::AdjustParam(int idx, double delta) {
    if (idx==0) {
        this->Kp += delta;
        cout<<"added "<<delta<<" to Kp"<<endl;
    } else if (idx==1) {
        this->Ki += delta;
        cout<<"added "<<delta<<" to Ki"<<endl;
    } else if (idx==2){
        this->Kd += delta;
        cout<<"added "<<delta<<" to Kd"<<endl;
    }
}