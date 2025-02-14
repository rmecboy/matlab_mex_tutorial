#include "mex.h"
#include "user_config.h"

static int16_t sim_init = 1;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    double *data_out[10];
    double data_in[10];
    
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[3] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[4] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[5] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[6] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[7] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[8] = mxCreateDoubleMatrix(1, 1, mxREAL);
    plhs[9] = mxCreateDoubleMatrix(1, 1, mxREAL);

    data_out[0] = mxGetPr(plhs[0]);
    data_out[1] = mxGetPr(plhs[1]);
    data_out[2] = mxGetPr(plhs[2]);
    data_out[3] = mxGetPr(plhs[3]);
    data_out[4] = mxGetPr(plhs[4]);
    data_out[5] = mxGetPr(plhs[5]);
    data_out[6] = mxGetPr(plhs[6]);
    data_out[7] = mxGetPr(plhs[7]);
    data_out[8] = mxGetPr(plhs[8]);
    data_out[9] = mxGetPr(plhs[9]);

    data_in[0] = *(mxGetPr(prhs[0]));
    data_in[1] = *(mxGetPr(prhs[1]));
    data_in[2] = *(mxGetPr(prhs[2]));
    data_in[3] = *(mxGetPr(prhs[3]));
    data_in[4] = *(mxGetPr(prhs[4]));
    data_in[5] = *(mxGetPr(prhs[5]));
    data_in[6] = *(mxGetPr(prhs[6]));
    data_in[7] = *(mxGetPr(prhs[7]));
    data_in[8] = *(mxGetPr(prhs[8]));
    data_in[9] = *(mxGetPr(prhs[9]));

    if (sim_init == 1)
    {
        sim_init = 0;
        task_init();
        mexPrintf("sim init done\n");
    }
    else if (sim_init == 0)
    {
        // mexPrintf("sim running\n");
        task_main();

        *data_out[0] = (double)u[0];
        *data_out[1] = (double)u[1];
        *data_out[2] = (double)u[2];
        *data_out[3] = (double)u[3];
        *data_out[4] = (double)x_current[0];//1
        *data_out[5] = (double)x_current[1];//2
        *data_out[6] = (double)x_current[2];
        *data_out[7] = (double)x_current[4];
        *data_out[8] = (double)x_current[6];
        *data_out[9] = (double)x_current[8];
    }
}