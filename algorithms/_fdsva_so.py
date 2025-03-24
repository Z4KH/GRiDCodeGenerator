import numpy as np
import copy 


def gen_fdsva_so_inner(self, use_thread_group = False): 
	# construct the boilerplate and function definition
    func_params = ["s_df2 are the second derivatives of forward dynamics WRT q,qd,tau", \
                "s_idsva_so are the second derivative tensors of inverse dynamics", \
                "s_Minv is the inverse mass matrix", \
                "s_df_du is the gradient of the forward dynamics", \
                "s_temp is the pointer to the shared memory needed of size: " + \
                            str(self.gen_fdsva_so_inner_temp_mem_size()), \
                "gravity is the gravity constant"]
    func_def_start = "void fdsva_so_inner("
    func_def_middle = "T *s_df2, T *s_idsva_so, T *s_Minv, T *s_df_du, "
    func_def_end = "T *s_temp, const T gravity) {"
    func_notes = ["Assumes works with IDSVA"]
    if use_thread_group:
        func_def_start = func_def_start.replace("(", "(cgrps::thread_group tgrp, ")
        func_params.insert(0,"tgrp is the handle to the thread_group running this function")
    func_def_middle, func_params = self.gen_insert_helpers_func_def_params(func_def_middle, func_params, -2)
    func_def = func_def_start + func_def_middle + func_def_end
    self.gen_add_func_doc("Second Order of Forward Dynamics with Spatial Vector Algebra", func_notes, func_params, None)
    self.gen_add_code_line("template <typename T>")
    self.gen_add_code_line("__device__")
    self.gen_add_code_line(func_def, True)

    n = self.robot.get_num_pos()

    #declare s_d2tau_dq, s_d2tau_dqd, s_d2tau_cross, s_dm_dq from idsvaso
    self.gen_add_code_line("T *s_d2tau_dq = &s_idsva_so[" + str(n*n*n*0) + "];" )
    self.gen_add_code_line("T *s_d2tau_dqd = &s_idsva_so[" + str(n*n*n*1) + "];" )
    self.gen_add_code_line("T *s_d2tau_cross = &s_idsva_so[" + str(n*n*n*2) + "];" )
    self.gen_add_code_line("T *s_dm_dq = &s_idsva_so[" + str(n*n*n*3) + "];" )
    #declare df_dq, df_dqd, fd_dtau from df_du 
    self.gen_add_code_line("T *s_df_dq = s_df_du; T *s_df_dqd = &s_df_du[" + str(n*n) + "];")
    
    self.gen_add_sync(use_thread_group)
    
    self.gen_add_code_line("T *dM_dqxfd_dq = s_temp;")

    self.gen_add_parallel_loop("ind",str(n*n*n),use_thread_group)
    #fill in mat mult of dM_dq + df_dq
    self.gen_add_code_line("int page = ind / " + str(n*n) + ";")
    self.gen_add_code_line("int row = ind % " + str(n) + ";")
    self.gen_add_code_line("int col = ind % " + str(n*n) + " / " + str(n) + ";")
    # self.gen_add_code_line("dM_dqxfd_dq[" + str(i*n*n) + "+ ind] = dot_prod<T,7,7,1>(&s_dm_dq" + str(i) + "[row], &s_df_dq[" + str(n) + "*row + col]);")
    self.gen_add_code_line("dM_dqxfd_dq[ind] = dot_prod<T," + str(n) + "," + str(n) + ",1>(&s_dm_dq[" + str(n*n) + "*page + row], &s_df_dq[" + str(n) + "*col]);")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    #rotation of dM_dq + df_dq
    NV = self.robot.get_num_vel()
    self.gen_add_code_line(f"T *rot_dM_dqxfd_dqd = &s_temp[{NV**3}];")  # Compute rotation after the original
    
    self.gen_add_parallel_loop("ind",str(NV**3),use_thread_group)
    self.gen_add_code_line(f"int page = ind / {NV**2};")
    self.gen_add_code_line(f"int row = ind % {NV};")
    self.gen_add_code_line(f"int col = ind / {NV} % {NV};")
    self.gen_add_code_line(f"rot_dM_dqxfd_dqd[{NV*NV}*col + row + {NV}*page] = dM_dqxfd_dq[ind];")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    self.gen_add_parallel_loop("ind",str(n*n*n),use_thread_group)
    # #big addition step for df2_dq
    self.gen_add_code_line("s_df2[ind] = s_d2tau_dq[ind] + dM_dqxfd_dq[ind] + rot_dM_dqxfd_dqd[ind];")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    self.gen_add_code_line("T *dM_dqxfd_dqd = s_temp;")

    self.gen_add_parallel_loop("ind",str(n*n*n),use_thread_group)
    #fill in mat mult of dM_dq + df_dqd
    self.gen_add_code_line("int page = ind / " + str(n*n) + ";")
    self.gen_add_code_line("int row = ind % " + str(n) + ";")
    self.gen_add_code_line("int col = ind % " + str(n*n) + " / " + str(n) + ";")
    # self.gen_add_code_line("dM_dqxfd_dq[" + str(i*n*n) + "+ ind] = dot_prod<T,7,7,1>(&s_dm_dq" + str(i) + "[row], &s_df_dq[" + str(n) + "*row + col]);")
    self.gen_add_code_line("dM_dqxfd_dqd[ind] = dot_prod<T," + str(n) + "," + str(n) + ",1>(&s_dm_dq[" + str(n*n) + "*page + row], &s_df_dqd[" + str(n) + "*col]);")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)


    self.gen_add_parallel_loop("ind",str(n*n*n),use_thread_group)
    #big addition step for df2_dq_qd
    self.gen_add_code_line("s_df2[" + str(n*n*n) + "+ ind] = s_d2tau_cross[ind] + dM_dqxfd_dqd[ind];")
    #load val for df2_dqd
    self.gen_add_code_line("s_df2[" + str(n*n*n*2) + "+ ind] = s_d2tau_dqd[ind];")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    #fill in mat mult of dM_dq + Minv
    #fix minv 
    for row_m in range(n):
        for col_m in range(n):
            if row_m > col_m:
                self.gen_add_code_line("s_Minv["+ str(row_m + col_m*n) + "] = s_Minv["+ str(row_m*n + col_m) + "];")

    self.gen_add_sync(use_thread_group)
    
    self.gen_add_parallel_loop("ind",str(n*n*n),use_thread_group)
    #fill in mat mult of dM_dq + df_dqd
    self.gen_add_code_line("int page = ind / " + str(n*n) + ";")
    self.gen_add_code_line("int row = ind % " + str(n) + ";")
    self.gen_add_code_line("int col = ind % " + str(n*n) + " / " + str(n) + ";")
    # self.gen_add_code_line("dM_dqxfd_dq[" + str(i*n*n) + "+ ind] = dot_prod<T,7,7,1>(&s_dm_dq" + str(i) + "[row], &s_df_dq[" + str(n) + "*row + col]);")
    self.gen_add_code_line("s_df2[" + str(n*n*n*3) + "+ ind] = dot_prod<T," + str(n) + "," + str(n) + ",1>(&s_dm_dq[" + str(n*n) + "*page + row], &s_Minv[" + str(n) + "*col]);")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)
    

    self.gen_add_code_line("T *s_df2_temp = &s_idsva_so[0];")
    

    self.gen_add_parallel_loop("ind",str(n*n*n*4),use_thread_group)
    #fill in mat mult of everything w minv
    self.gen_add_code_line("int page = ind / " + str(n*n) + ";")
    self.gen_add_code_line("int row = ind % " + str(n) + ";")
    self.gen_add_code_line("int col = ind % " + str(n*n) + " / " + str(n) + ";")
    self.gen_add_code_line("s_df2_temp[ind] = dot_prod<T," + str(n) + "," + str(n) + ",1>(&s_Minv[row], &s_df2[" + str(n*n) + "*page + " + str(n) + "*col]);")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    self.gen_add_parallel_loop("ind",str(n*n*n*4),use_thread_group)
    #mult everything by -1
    self.gen_add_code_line("s_df2[ind] = s_df2_temp[ind];")
    self.gen_add_code_line("s_df2[ind] *= (-1);")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    self.gen_add_end_function()

    

    
def gen_fdsva_so_inner_temp_mem_size(self):
    n = self.robot.get_num_pos()
    return n*n*n*4*3
    
def gen_fdsva_so_inner_function_call(self, use_thread_group = False, updated_var_names = None):
    var_names = dict( \
        s_df2_name = "s_df2", \
        s_idsva_so_name = "s_idsva_so", \
        s_Minv_name = "s_Minv", \
        s_df_du_name = "s_df_du", \
        s_temp_name = "s_mem", \
        gravity_name = "gravity"
    )
    if updated_var_names is not None:
        for key,value in updated_var_names.items():
            var_names[key] = value
    fdsva_so_code_start = "fdsva_so_inner<T>(" + var_names["s_df2_name"] + ", " + var_names["s_idsva_so_name"] + ", " + var_names["s_Minv_name"] + ", " + var_names["s_df_du_name"] + ", "
    fdsva_so_code_end = var_names["s_temp_name"] + ", " + var_names["gravity_name"] + ");"
    if use_thread_group:
        id_code_start = id_code_start.replace("(","(tgrp, ")
    fdsva_so_code_middle = self.gen_insert_helpers_function_call()
    fdsva_so_code = fdsva_so_code_start + fdsva_so_code_middle + fdsva_so_code_end
    self.gen_add_code_line(fdsva_so_code)

def gen_fdsva_so_device_temp_mem_size(self):
    n = self.robot.get_num_pos()
    wrapper_size = self.gen_topology_helpers_size() 
    return self.gen_fdsva_so_inner_temp_mem_size() + wrapper_size

def gen_fdsva_so_device(self, use_thread_group = False):
    n = self.robot.get_num_pos()
    # construct the boilerplate and function definition
    func_params = ["s_q is the vector of joint positions", \
                   "s_qd is the vector of joint velocities", \
                   "s_qdd is the vector of joint accelerations", \
                   "s_tau is the vector of joint torques", \
                   "d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)", \
                   "gravity is the gravity constant"]
    func_notes = []
    func_def_start = "void fdsva_so_device("
    func_def_middle = "const T *s_q, const T *s_qd, const T *s_qdd, const T *s_tau,"
    func_def_end = "const robotModel<T> *d_robotModel, const T gravity) {"
    if use_thread_group:
        func_def_start += "cgrps::thread_group tgrp, "
        func_params.insert(0,"tgrp is the handle to the thread_group running this function")
    func_def = func_def_start + func_def_middle + func_def_end

    # then generate the code
    self.gen_add_func_doc("Compute the FDSVA_SO (Second Order of Forward Dyamics with Spacial Vector Algebra)",\
                          func_notes,func_params,None)
    self.gen_add_code_line("template <typename T>")
    self.gen_add_code_line("__device__")
    self.gen_add_code_line(func_def, True)

    # add the shared memory variables
    shared_mem_size = self.gen_fdsva_so_device_temp_mem_size() if not self.use_dynamic_shared_mem_flag else None
    self.gen_XImats_helpers_temp_shared_memory_code(shared_mem_size)

    self.gen_add_code_line("extern __shared__ T s_df2[" + str(3*n*3*n*n) + "];")
    
    # then load/update XI and run the algo
    self.gen_load_update_XImats_helpers_function_call(use_thread_group)
    self.gen_fdsva_so_inner_function_call(use_thread_group)
    self.gen_add_end_function()

def gen_fdsva_so_kernel(self, use_thread_group = False, single_call_timing = False):
    n = self.robot.get_num_pos()
    # define function def and params
    func_params = ["d_q_qd_u is the vector of joint positions, velocities, torques", \
                    "stride_q_qd_u is the stride between each q, qd, qdd", \
                    "d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)", \
                    "gravity is the gravity constant", \
                    "num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)"]
    func_notes = []
    func_def_start = "void fdsva_so_kernel(T *d_df2, const T *d_q_qd_u, const int stride_q_qd_u, "
    func_def_end = "const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {"
    func_def = func_def_start + func_def_end
    if single_call_timing:
        func_def = func_def.replace("kernel(", "kernel_single_timing(")
    
    # then generate the code
    self.gen_add_func_doc("Compute the FDSVA_SO (Second Order of Forward Dynamics with Spacial Vector Algebra)", \
                            func_notes, func_params, None)
    self.gen_add_code_line("template <typename T>")
    self.gen_add_code_line("__global__")
    self.gen_add_code_line(func_def, True)

    # add shared memory variables
    shared_mem_vars = ["__shared__ T s_df2[" + str(4*n*n*n) + "];", \
                        "__shared__ T s_q_qd_u[4*" + str(n) + "]; T *s_q = s_q_qd_u; T *s_qd = &s_q_qd_u[" + str(n) + "]; T *s_u = &s_q_qd_u[2 * " + str(n) + "];",\
                        f"__shared__ T s_Minv[{n*n}];", \
                        f"__shared__ T s_qdd[{n}];", \
                        f"__shared__ T s_df_du[{2*n*n}];", \
                        f'__shared__ T s_idsva_so[{n*n*n*4}]; __shared__ T s_mem[{self.gen_idsva_so_inner_temp_mem_size()}];']
    self.gen_add_code_lines(shared_mem_vars)
    shared_mem_size = self.gen_fdsva_so_inner_temp_mem_size() if not self.use_dynamic_shared_mem_flag else None
    self.gen_XImats_helpers_temp_shared_memory_code(shared_mem_size)
    if use_thread_group:
        self.gen_add_code_line("cgrps::thread_group tgrp = TBD;")
    if not single_call_timing:
        # load to shared mem and loop over blocks to compute all requested comps
        self.gen_add_parallel_loop("k","NUM_TIMESTEPS",use_thread_group,block_level = True)
        self.gen_kernel_load_inputs("q_qd_u","stride_q_qd_u",str(3*n),use_thread_group)
        # compute
        self.gen_add_code_line("// compute")
        self.gen_load_update_XImats_helpers_function_call(use_thread_group)
        # Need Minv, FD Gradient, IDSVA-SO
        self.gen_direct_minv_inner_function_call(use_thread_group)
        self.gen_add_code_line("forward_dynamics_inner<T>(s_qdd, s_q, s_qd, s_u, s_XImats, s_mem, gravity);")
        self.gen_forward_dynamics_gradient_device_function_call()
        self.gen_idsva_so_inner_function_call(use_thread_group)
        self.gen_fdsva_so_inner_function_call(use_thread_group)
        self.gen_add_sync(use_thread_group)
        # save to global
        self.gen_kernel_save_result("df2","1",str(4*n*n*n),use_thread_group)
        self.gen_add_end_control_flow()
    else:
        # repurpose NUM_TIMESTEPS for number of timing reps
        self.gen_kernel_load_inputs_single_timing("q_qd_u",str(3*n),use_thread_group)
        # then compute in loop for timing
        self.gen_add_code_line("// compute with NUM_TIMESTEPS as NUM_REPS for timing")
        self.gen_add_code_line("for (int rep = 0; rep < NUM_TIMESTEPS; rep++){", True)
        self.gen_load_update_XImats_helpers_function_call(use_thread_group)
        self.gen_direct_minv_inner_function_call(use_thread_group)
        self.gen_add_code_line("forward_dynamics_inner<T>(s_qdd, s_q, s_qd, s_u, s_XImats, s_mem, gravity);")
        self.gen_forward_dynamics_gradient_device_function_call()
        self.gen_idsva_so_inner_function_call(use_thread_group)
        self.gen_fdsva_so_inner_function_call(use_thread_group)
        self.gen_add_end_control_flow()
        # save to global
        self.gen_kernel_save_result_single_timing("df2",str(4*n*n*n),use_thread_group)
    self.gen_add_end_function()

def gen_fdsva_so_host(self, mode = 0):
    n = self.robot.get_num_pos()
    # default is to do the full kernel call -- options are for single timing or compute only kernel wrapper
    single_call_timing = True if mode == 1 else False
    compute_only = True if mode == 2 else False

    # define function def and params
    func_params = ["hd_data is the packaged input and output pointers", \
                   "d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)", \
                   "gravity is the gravity constant,", \
                   "num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)", \
                   "streams are pointers to CUDA streams for async memory transfers (if needed)"]
    func_notes = []
    func_def_start = "void fdsva_so(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,"
    func_def_end =   "                      const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {"
    if single_call_timing:
        func_def_start = func_def_start.replace("(", "_single_timing(")
        func_def_end = "              " + func_def_end
    if compute_only:
        func_def_start = func_def_start.replace("(", "_compute_only(")
        func_def_end = "             " + func_def_end.replace(", cudaStream_t *streams", "")
    # then generate the code
    self.gen_add_func_doc("Compute the FDSVA_SO (Second Order of Forward Dynamics with Spacial Vector Algebra)",\
                          func_notes,func_params,None)
    self.gen_add_code_line("template <typename T>")
    self.gen_add_code_line("__host__")
    self.gen_add_code_line(func_def_start)
    self.gen_add_code_line(func_def_end, True)

    func_call_start = "fdsva_so_kernel<T><<<block_dimms,thread_dimms,FDSVA_SO_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_df2,hd_data->d_q_qd_u,stride_q_qd_qdd,"
    func_call_end = "d_robotModel,gravity,num_timesteps);"
    self.gen_add_code_line("int stride_q_qd_qdd = 3*NUM_JOINTS;")
    if single_call_timing:
        func_call_start = func_call_start.replace("kernel<T>","kernel_single_timing<T>")
    if not compute_only:
        # start code with memory transfer
        self.gen_add_code_lines(["// start code with memory transfer", \
                                 "gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd_qdd" + \
                                    ("*num_timesteps" if not single_call_timing else "") + "*sizeof(T),cudaMemcpyHostToDevice,streams[0]));", \
                                 "gpuErrchk(cudaDeviceSynchronize());"])
    # then compute:
    self.gen_add_code_line("// call the kernel")
    func_call = func_call_start + func_call_end
    func_call_code = [func_call, "gpuErrchk(cudaDeviceSynchronize());"]
    # wrap function call in timing (if needed)
    if single_call_timing:
        func_call_code.insert(0,"struct timespec start, end; clock_gettime(CLOCK_MONOTONIC,&start);")
        func_call_code.append("clock_gettime(CLOCK_MONOTONIC,&end);")
    self.gen_add_code_lines(func_call_code)
    if not compute_only:
        # then transfer memory back
        self.gen_add_code_lines(["// finally transfer the result back", \
                                "gpuErrchk(cudaMemcpy(hd_data->h_df2,hd_data->d_df2," + \
                                ("num_timesteps*" if not single_call_timing else "") + str(4*n**3) + "*sizeof(T),cudaMemcpyDeviceToHost));",
                                "gpuErrchk(cudaDeviceSynchronize());"])
    # finally report out timing if requested
    if single_call_timing:
        self.gen_add_code_line("printf(\"Single Call FDSVA_SO %fus\\n\",time_delta_us_timespec(start,end)/static_cast<double>(num_timesteps));")
    self.gen_add_end_function()

def gen_fdsva_so(self, use_thread_group = False):
    # first generate the inner helper
    self.gen_fdsva_so_inner(use_thread_group)
    # then generate the device wrapper
    # self.gen_fdsva_so_device(use_thread_group) TODO: fix this
    # then generate the kernels
    self.gen_fdsva_so_kernel(use_thread_group, True)
    self.gen_fdsva_so_kernel(use_thread_group, False)
    # then generate the host wrappers
    self.gen_fdsva_so_host(0)
    self.gen_fdsva_so_host(1)
    self.gen_fdsva_so_host(2)
    
