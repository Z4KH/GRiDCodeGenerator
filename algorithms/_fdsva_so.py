MEMORY_THRESHOLD = 8 # Max num joints for shared mem allocation of result


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

    n = self.robot.get_num_vel()
    NV = self.robot.get_num_vel()

    self.gen_add_code_line('// Second Derivatives of Inverse Dynamics')
    self.gen_add_code_line("T *d2tau_dqdq = &s_idsva_so[" + str(n*n*n*0) + "];" )
    self.gen_add_code_line("T *d2tau_dvdv = &s_idsva_so[" + str(n*n*n*1) + "];" )
    self.gen_add_code_line("T *d2tau_dvdq = &s_idsva_so[" + str(n*n*n*2) + "];" )
    self.gen_add_code_line("T *dM_dq = &s_idsva_so[" + str(n*n*n*3) + "];" )
    self.gen_add_code_line('\n\n')
    self.gen_add_code_line('// First Derivatives of Forward Dynamics')
    self.gen_add_code_line("T *s_df_dq = s_df_du; T *s_df_dqd = &s_df_du[" + str(n*n) + "];")
    self.gen_add_code_line('\n\n')
    self.gen_add_code_line('// Second Derivatives of Forward Dynamics')
    self.gen_add_code_line('T *d2a_dqdq = s_df2;')
    self.gen_add_code_line('T *d2a_dvdv = &s_df2[' + str(n*n*n) + '];')
    self.gen_add_code_line('T *d2a_dvdq = &s_df2[' + str(2*n*n*n) + '];')
    self.gen_add_code_line('T *d2a_dtdq = &s_df2[' + str(3*n*n*n) + '];')
    self.gen_add_code_line('\n\n')
    self.gen_add_code_line('// Temporary Variables')
    self.gen_add_code_line(f'T *inner_dq = s_temp; // Inner term for d2a_dqdq (d2tau_dqdq + dM_dq*da_dq + (dM_dq*da_dq)^R)')
    self.gen_add_code_line(f'T *inner_cross = inner_dq + {n**3}; // Inner term for d2a_dvdq (dM_dq*Minv)')
    self.gen_add_code_line(f'T *inner_tau = inner_cross + {n**3}; // Inner term for d2a_dtdq (d2tau_dvdq + dM_dq*da_dv)')
    self.gen_add_code_line(f'T *rot_dq = inner_tau + {n**3}; // Rotated (dM_dq*da_dq)^R term used to compute inner_dq')
    self.gen_add_code_line(f'\n\n')

    # Start inner term for d2a_dqdq
    self.gen_add_code_line('// Start inner term for d2a_dqdq & Fill out Minv')
    self.gen_add_parallel_loop("ind",str(n**3 + n*n),use_thread_group)
    self.gen_add_code_line(f'int i = ind / {n*n} % {n}; int j = ind / {n} % {n}; int k = ind % {n};')
    self.gen_add_code_line(f'if (ind < {n**3}) {{', True)
    self.gen_add_code_line(f'inner_dq[ind] = dot_prod<T, {n}, 1, 1>(&dM_dq[{n*n}*i + {n}*k], &s_df_dq[{n}*j]);')
    self.gen_add_code_line(f'rot_dq[i*{n*n} + k*{n} + j] = inner_dq[ind];')
    self.gen_add_end_control_flow()
    self.gen_add_code_line(f'else if (k > j) s_Minv[j*{n} + k] = s_Minv[k*{n} + j];')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)
    
    # 3Dx2D Tensor Computation defined as iLk,Lj->ijk
    self.gen_add_code_line('// Compute relevant inner subterms in parallel')
    self.gen_add_parallel_loop("ind",str(3*n**3),use_thread_group)
    self.gen_add_code_line(f'int i = ind / {n*n} % {n}; int j = ind / {n} % {n}; int k = ind % {n};')
    self.gen_add_code_line(f'if (ind < {n**3}) inner_dq[ind] += rot_dq[ind] + d2tau_dqdq[ind]; // Started with dM_dq*da_dq')
    self.gen_add_code_line(f'else if (ind < {2*n**3}) inner_cross[i*{n*n} + k*{n} + j] = dot_prod<T, {n}, 1, 1>(&dM_dq[{n*n}*i + {n}*k], &s_df_dqd[{n}*j]) + d2tau_dvdq[i*{n*n} + k*{n} + j];')
    self.gen_add_code_line(f'else inner_tau[i*{n*n} + k*{n} + j] = dot_prod<T, {n}, 1, 1>(&dM_dq[{n*n}*i + {n}*k], &s_Minv[{n}*j]);')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # 2Dx3D tensor computation defined as iL,Ljk->ijk
    self.gen_add_code_line('// Multiply by -Minv to finish algorithm')
    self.gen_add_parallel_loop("ind",str(4*n**3),use_thread_group)
    self.gen_add_code_line(f'int i = ind / {n*n} % {n}; int j = ind / {n} % {n}; int k = ind % {n};')
    self.gen_add_code_line(f'if (ind < {n**3}) d2a_dqdq[i*{n*n} + j + k*{n}] = -dot_prod<T, {n}, {n}, {n*n}>(&s_Minv[i], &inner_dq[j + k*{n}]);')
    self.gen_add_code_line(f'else if (ind < {2*n**3}) d2a_dvdq[i*{n*n} + j + k*{n}] = -dot_prod<T, {n}, {n}, {n*n}>(&s_Minv[i], &inner_cross[j + k*{n}]);')
    self.gen_add_code_line(f'else if (ind < {3*n**3}) d2a_dvdv[i*{n*n} + j + k*{n}] = -dot_prod<T, {n}, {n}, {n*n}>(&s_Minv[i], &d2tau_dvdv[j + k*{n}]);')
    self.gen_add_code_line(f'else d2a_dtdq[i*{n*n} + j + k*{n}] = -dot_prod<T, {n}, {n}, {n*n}>(&s_Minv[i], &inner_tau[j + k*{n}]);')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    self.gen_add_end_function()

def gen_fdsva_so_device_temp_mem_size(self):
    # Same as idsva_so because idsva_so is called and takes more memory
    return self.gen_fdsva_so_inner_temp_mem_size()
    
def gen_fdsva_so_inner_temp_mem_size(self):
    NV = self.robot.get_num_vel()
    jids_a, ancestors = self.robot.get_jid_ancestor_ids(include_joint=True)
    return int(36 * NV * 10 + 30 * NV + 6 + len(jids_a)*36)
    
def gen_fdsva_so_inner_function_call(self, use_thread_group = False, updated_var_names = None):
    var_names = dict( \
        s_df2_name = "s_df2", \
        s_idsva_so_name = "s_idsva_so", \
        s_Minv_name = "s_Minv", \
        s_df_du_name = "s_df_du", \
        s_temp_name = "s_temp", \
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

def gen_fdsva_so_device(self, use_thread_group = False):
    n = self.robot.get_num_pos()
    # construct the boilerplate and function definition
    func_params = ["s_df2 is the second derivatives of forward dynamics WRT q,qd,tau", \
                   "s_df_du is a pointer to memory for the derivative of forward dynamics WRT q,qd of size 2*NUM_JOINTS*NUM_JOINTS = " + str(2*n*n), \
                   "s_q is the vector of joint positions", \
                   "s_qd is the vector of joint velocities", \
                   "s_u is the vector of joint control inputs", \
                   "d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)", \
                   "gravity is the gravity constant"]
    func_notes = []
    func_def_start = "void fdsva_so_device("
    func_def_middle = "T *s_df2, T *s_df_du, const T *s_q, const T *s_qd, const T *s_u, "
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
    self.gen_add_code_line(f"__shared__ T s_Minv[{n*n}];")
    self.gen_add_code_line(f"__shared__ T s_qdd[{n}];")
    self.gen_add_code_line(f"__shared__ T s_idsva_so[{n*n*n*4}];")
    shared_mem_size = self.gen_fdsva_so_device_temp_mem_size()
    self.gen_XImats_helpers_temp_shared_memory_code(shared_mem_size)
    
    # then load/update XI and run the algo
    self.gen_load_update_XImats_helpers_function_call(use_thread_group)
    self.gen_direct_minv_inner_function_call(use_thread_group)
    self.gen_add_code_line(f"forward_dynamics_inner<T>(s_qdd, s_q, s_qd, s_u, s_XImats, s_temp, gravity);")
    self.gen_add_sync(use_thread_group)
    self.gen_forward_dynamics_gradient_device_function_call()
    self.gen_idsva_so_inner_function_call(use_thread_group)
    self.gen_fdsva_so_inner_function_call(use_thread_group)
    self.gen_add_end_function()

def gen_fdsva_so_kernel(self, use_thread_group = False, single_call_timing = False):
    n = self.robot.get_num_pos()
    # define function def and params
    func_params = ["d_df2 is the second derivatives of forward dynamics WRT q,qd,tau", \
                    "d_q_qd_u is the vector of joint positions, velocities, torques", \
                    "stride_q_qd_u is the stride between each q, qd, qdd", \
                    "d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)", \
                    "gravity is the gravity constant", \
                    "num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)"]
    func_notes = []
    func_def_start = "void fdsva_so_kernel(T *d_df2, const T *d_q_qd_u, const int stride_q_qd_u, "
    if n > MEMORY_THRESHOLD:
        func_params.append("d_idsva_so is the pointer to the idsva_so output tensor in global memory")
        func_def_start += "T *d_idsva_so, "
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
    shared_mem_vars = ["__shared__ T s_q_qd_u[4*" + str(n) + "]; T *s_q = s_q_qd_u; T *s_qd = &s_q_qd_u[" + str(n) + "]; T *s_u = &s_q_qd_u[2 * " + str(n) + "];",\
                        f"__shared__ T s_Minv[{n*n}];", \
                        f"__shared__ T s_qdd[{n}];", \
                        f"__shared__ T s_df_du[{2*n*n}];"]
    if n <= MEMORY_THRESHOLD:
        shared_mem_vars.append(f"__shared__ T s_idsva_so[{n*n*n*4}];")
        shared_mem_vars.append(f"__shared__ T s_df2[" + str(4*n*n*n) + "];")
    self.gen_add_code_lines(shared_mem_vars)
    self.gen_XImats_helpers_temp_shared_memory_code(self.gen_idsva_so_inner_temp_mem_size())
    if use_thread_group:
        self.gen_add_code_line("cgrps::thread_group tgrp = TBD;")
    fd_start = "forward_dynamics_inner<T>(s_qdd, s_q, s_qd, s_u, "
    fd_end = "s_temp, gravity);"
    fd_start, _ = self.gen_insert_helpers_func_def_params(fd_start, [], -2)
    if 'T *' in fd_start: fd_start = fd_start.replace("T *","")
    if 'int *' in fd_start: fd_start = fd_start.replace("int *","")
    if not single_call_timing:
        # load to shared mem and loop over blocks to compute all requested comps
        self.gen_add_parallel_loop("k","NUM_TIMESTEPS",use_thread_group,block_level = True)
        self.gen_kernel_load_inputs("q_qd_u","stride_q_qd_u",str(3*n),use_thread_group)
        if n > MEMORY_THRESHOLD: 
            self.gen_add_code_line(f'T *s_df2 = &d_df2[k*{4*n**3}];')
            self.gen_add_code_line(f'T *s_idsva_so = &d_idsva_so[k*{4*n**3}];')
        # compute
        self.gen_add_code_line("// compute")
        self.gen_load_update_XImats_helpers_function_call(use_thread_group)
        # Need Minv, FD Gradient, IDSVA-SO
        self.gen_direct_minv_inner_function_call(use_thread_group)
        self.gen_add_code_line(fd_start + fd_end)
        self.gen_add_sync(use_thread_group)
        self.gen_forward_dynamics_gradient_device_function_call()
        self.gen_idsva_so_inner_function_call(use_thread_group)
        self.gen_fdsva_so_inner_function_call(use_thread_group)
        self.gen_add_sync(use_thread_group)
        # save to global
        if n <= MEMORY_THRESHOLD: self.gen_kernel_save_result("df2",f"{4*n**3}",str(4*n*n*n),use_thread_group)
        self.gen_add_end_control_flow()
    else:
        # repurpose NUM_TIMESTEPS for number of timing reps
        self.gen_kernel_load_inputs_single_timing("q_qd_u",str(3*n),use_thread_group)
        # then compute in loop for timing
        self.gen_add_code_line("// compute with NUM_TIMESTEPS as NUM_REPS for timing")
        self.gen_add_code_line("for (int rep = 0; rep < NUM_TIMESTEPS; rep++){", True)
        if n > MEMORY_THRESHOLD: 
            self.gen_add_code_line(f'T *s_df2 = &d_df2[rep*{4*n**3}];')
            self.gen_add_code_line(f'T *s_idsva_so = &d_idsva_so[rep*{4*n**3}];')
        self.gen_load_update_XImats_helpers_function_call(use_thread_group)
        self.gen_direct_minv_inner_function_call(use_thread_group)
        self.gen_add_code_line(fd_start + fd_end)
        self.gen_add_sync(use_thread_group)
        self.gen_forward_dynamics_gradient_device_function_call()
        self.gen_idsva_so_inner_function_call(use_thread_group, updated_var_names = dict(s_mem_name = "s_temp"))
        self.gen_fdsva_so_inner_function_call(use_thread_group)
        self.gen_add_end_control_flow()
        # save to global
        if n <= MEMORY_THRESHOLD: self.gen_kernel_save_result_single_timing("df2",str(4*n*n*n),use_thread_group)
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
    if n > MEMORY_THRESHOLD:
        func_call_start += "hd_data->d_idsva_so,"
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
    self.gen_fdsva_so_device(use_thread_group)
    # then generate the kernels
    self.gen_fdsva_so_kernel(use_thread_group, True)
    self.gen_fdsva_so_kernel(use_thread_group, False)
    # then generate the host wrappers
    self.gen_fdsva_so_host(0)
    self.gen_fdsva_so_host(1)
    self.gen_fdsva_so_host(2)
    
