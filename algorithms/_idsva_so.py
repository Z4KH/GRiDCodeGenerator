def gen_idsva_so_inner_temp_mem_size(self):
    # TODO - much more than this
    NV = self.robot.get_num_vel()
    return 4*NV**3

def gen_idsva_so_inner_function_call(self, use_thread_group = False, use_qdd_input = False, updated_var_names = None):
    var_names = dict( \
        s_idsva_so_name = "s_idsva_so", \
        s_q_name = "s_q", \
        s_qd_name = "s_qd", \
        s_qdd_name = "s_qdd", \
        s_temp_name = "s_temp", \
        s_a_name = "s_a", \
        s_b_name = "s_b", \
        gravity_name = "gravity"
    )
    if updated_var_names is not None:
        for key,value in updated_var_names.items():
            var_names[key] = value
    id_so_code_start = "idsva_so_inner<T>(" + var_names["s_idsva_so_name"] + ", " + var_names["s_q_name"] + ", " + var_names["s_qd_name"] + ", "
    id_so_code_middle = self.gen_insert_helpers_function_call()
    id_so_code_end = f'{var_names["s_a_name"]}, {var_names["s_b_name"]}, ' + var_names["s_temp_name"] + ", " + var_names["gravity_name"] + ");"
    if use_thread_group:
        id_so_code_start = id_so_code_start.replace("(","(tgrp, ")
    id_so_code = id_so_code_start + id_so_code_middle + id_so_code_end
    self.gen_add_code_line(id_so_code)

def gen_idsva_so_inner(self, use_thread_group = False, use_qdd_input = False):
    """
    Generates the inner device function to compute the second order
    idsva.
    """
    NV = self.robot.get_num_vel()
    NJ = self.robot.get_num_joints()

    # construct the boilerplate and function definition
    func_params = ["s_idsva_so is a pointer to memory for the final result of size 4*NUM_JOINTS*NUM_JOINTS*NUM_JOINTS = " + str(4*NV**3), \
                   "s_q is the vector of joint positions", \
                   "s_qd is the vector of joint velocities", \
# TODO - shared memory size
                   "s_temp is a pointer to helper shared memory of size  = " + \
                            str(self.gen_idsva_so_inner_temp_mem_size()), \
                   "gravity is the gravity constant"]
    func_def_start = "void idsva_so_inner(T *s_idsva_so, const T *s_q, const T *s_qd, "
    func_def_end = "T *s_temp, const T gravity) {"
    func_def_start, func_params = self.gen_insert_helpers_func_def_params(func_def_start, func_params, -2)
    func_notes = ["Assumes s_XImats is updated already for the current s_q"]
    if use_thread_group:
        func_def_start = func_def_start.replace("(", "(cgrps::thread_group tgrp, ")
        func_params.insert(0,"tgrp is the handle to the thread_group running this function")
    func_def = func_def_start + func_def_end
    # then generate the code
    self.gen_add_func_doc("Computes the second order derivatives of inverse dynamics",func_notes,func_params,None)
    self.gen_add_code_line("template <typename T>")
    self.gen_add_code_line("__device__")
    self.gen_add_code_line(func_def, True)

    pointers = [
        "T *s_d2tau_dq = s_idsva_so;"
        f'T *s_d2tau_dqd = s_idsva_so[{NV}];'
        f'T *s_d2tau_cross = s_idsva_so[{NV**2}];'
        f'T *s_dM_dq = s_idsva_so[{NV**3}];'
        "int cur;",
        "int factor;",
        "T *p1;",
        "T *p2;",
        "T *p3;",
        "T *I = s_XImats + 36*NUM_JOINTS;",
        "T *Xup0 = s_b;",
        "T *Xdown0 = s_b + 36*NUM_JOINTS;",
        "T *BC = Xup0;",
        "T *temp_b = Xdown0;",
        "T *I_Xup0 = s_b + 36*NUM_JOINTS*2;",
        "T *IC = s_b + 36*NUM_JOINTS*3;",
        "T *t1_Sj = s_b + (6*NUM_JOINTS);",
        "T *t4_Sd = s_b +2*(6*NUM_JOINTS);",
        "T *t1_Sdd = s_b +3*(6*NUM_JOINTS);",
        "T *S = s_a;",
        "T *vJ = s_a + (6*NUM_JOINTS);",
        "T *S_qdd = s_a + 2*(6*NUM_JOINTS);",
        "T *v = s_a + 3*(6*NUM_JOINTS);",
        "T *psid = s_a + 4*(6*NUM_JOINTS);",
        "T *aJ = s_a + 5*(6*NUM_JOINTS);",
        "T *Sj1 = s_a + 6*(6*NUM_JOINTS);",
        "T *IC_v = vJ;",
        "T *a = vJ;",
        "T *crm_v = s_a + 7*(6*NUM_JOINTS);",
        "T *Sdd1 = crm_v;",
        "T *Sdd2 = s_a + 8*(6*NUM_JOINTS);",
        "T *f1 = s_a + 9*(6*NUM_JOINTS);",
        "T *f2 = s_a + 10*(6*NUM_JOINTS);",
        "T *f = a;",
        "T *psidd = s_a + 11*(6*NUM_JOINTS);",
        "T *Sj = S_qdd;",
        "T *t1 = Sj1;",
        "T *BC_S = Sdd1;",
        "T *IC_Sj = Sdd2;",
        "T *t4 = f1;",
        "T *IC_Sdd = f2;",
        "T *BC_Sd = s_a + 12*(6*NUM_JOINTS);",
        "T *icrf_f_S = v;",
        "T *t2 = f;",
        "T *t3 = aJ;",
        "T *Sd = s_c;",

        "// For the Backward Pass",
        "T *Bic_phii = s_c + 36*NUM_JOINTS;",

        "// Calculating for Temporary Variables",
        "T *D1 = s_D;",
        "T *D2 = D1 + 36*NUM_JOINTS;",
        "T *D3 = D2 + 36*NUM_JOINTS;",
        "T *D4 = D3 + 36*NUM_JOINTS;"
    ]

    self.gen_add_code_lines(pointers)

    # identity matrices
    self.gen_add_code_line("\n")
    self.gen_add_code_line("// Set identity Matrices")
    self.gen_add_parallel_loop("index", str(72*NV), use_thread_group)
    self.gen_add_code_line("if (index%6 == (index%36)/6) s_b[index] = 1;")
    self.gen_add_code_line("else s_b[index] = 0;")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # xup
    self.gen_add_code_line("\n")
    self.gen_add_code_line("// Compute Xup")
    for j in range(NJ):
        self.gen_add_code_line(f'cur = {j}*36;')
        self.gen_add_parallel_loop("index",'36',use_thread_group)
        if j == 0: # base
            self.gen_add_code_line('Xup0[index] = s_XImats[index];')
        else: 
            self.gen_add_code_line('matmul<T>(index, &Xup0[cur-36], &s_XImats[cur], &Xup0[cur], 36, 0);')
        self.gen_add_end_control_flow()
        self.gen_add_sync(use_thread_group)

    self.gen_add_parallel_loop("index",str(NJ*36),use_thread_group)
    self.gen_add_code_line("cur = (index/36)*36;")
    #TODO why cur = 0 in grid
    self.gen_add_code_line("matmul<T>(index, &Xup0[cur], I, I_Xup0, 36, 0);")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    self.gen_add_parallel_loop('index',str(36*NJ),use_thread_group)
    self.gen_add_code_line('cur = (index/36)*36;')
    #TODO why cur = 0 in grid
    self.gen_add_code_line("matmul_trans<T>(index, &Xup0[cur], &I_Xup0[cur], IC, 'f');")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    self.gen_add_end_function()
        



def gen_idsva_so_device_temp_mem_size(self, compute_c = False):
    pass

def gen_idsva_so_device(self, use_thread_group = False, use_qdd_input = False, single_call_timing=False):
    NV = self.robot.get_num_vel()
    # construct the boilerplate and function definition
    func_params = ["s_idsva_so is a pointer to memory for the final result of size 4*NUM_JOINTS*NUM_JOINTS*NUM_JOINTS = " + str(4*NV**3), \
                   "s_q is the vector of joint positions", \
                   "s_qd is the vector of joint velocities", \
                   "d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)", \
                   "gravity is the gravity constant"]
    func_def_start = "void idsva_so_device(T *s_idsva_so, const T *s_q, const T *s_qd, "
    func_def_end = "const robotModel<T> *d_robotModel, const T gravity) {"
    func_notes = []
    if use_thread_group:
        func_def_start += "cgrps::thread_group tgrp, "
        func_params.insert(0,"tgrp is the handle to the thread_group running this function")
    if use_qdd_input:
        func_def_start += "const T *s_qdd, "
        func_params.insert(-2,"s_qdd is the vector of joint accelerations")
    else:
        func_notes.append("optimized for qdd = 0")
    func_def = func_def_start + func_def_end
    self.gen_add_func_doc("Computes the second order derivates of idsva",func_notes,func_params,None)
    self.gen_add_code_line("template <typename T>")
    self.gen_add_code_line("__device__")
    self.gen_add_code_line(func_def, True)
    # add the shared memory variables
    shared_pointers = [
        # TODO - figure out where s_a, and s_b get their sizes from
        "__shared__ T s_a[1588];",
        "__shared__ T s_b[1268];",
        f"__shared__ T s_idsva_so[{4*NV**3}];",
    ]   
    shared_mem_size = self.gen_idsva_so_inner_temp_mem_size() if not self.use_dynamic_shared_mem_flag else None
    self.gen_XImats_helpers_temp_shared_memory_code(shared_mem_size)
    # then load/update XI and run the algo
    self.gen_load_update_XImats_helpers_function_call(use_thread_group)
    self.gen_idsva_so_inner_function_call(use_thread_group)
    self.gen_add_end_function()

def gen_idsva_so_kernel(self, use_thread_group = False, use_qdd_input = False, single_call_timing = False):
    NUM_POS = self.robot.get_num_pos()
    n = self.robot.get_num_vel()
    NJ = self.robot.get_num_joints()
    # define function def and params
    func_params = ["d_idsva_so is a pointer to memory for the final result of size 4*NUM_JOINTS*NUM_JOINTS*NUM_JOINTS = " + str(4*n**3), \
                   "d_q_dq is the vector of joint positions and velocities", \
                   "stride_q_qd is the stide between each q, qd", \
                   "d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)", \
                   "gravity is the gravity constant", \
                   "num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)"]
    func_notes = []
    func_def_start = "void idsva_so_kernel(T *d_idsva_so, const T *d_q_qd, const int stride_q_qd, "
    func_def_end = "const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {"
    if use_qdd_input:
        func_def_start += "const T *d_qdd, "
        func_params.insert(-2,"d_qdd is the vector of joint accelerations")
    else:
        func_notes.append("optimized for qdd = 0")
    func_def = func_def_start + func_def_end
    if single_call_timing:
        func_def = func_def.replace("kernel(", "kernel_single_timing(")
    # then generate the code
    self.gen_add_func_doc("Computes the second order derivatives of inverse dynamics",func_notes,func_params,None)
    self.gen_add_code_line("template <typename T>")
    self.gen_add_code_line("__global__")
    self.gen_add_code_line(func_def, True)
    # add shared memory variables
    shared_mem_vars = ["__shared__ T s_q_qd[" + str(n + NUM_POS) + "]; T *s_q = s_q_qd; T *s_qd = &s_q_qd[" + str(NUM_POS) + "];", \
                    f"__shared__ T s_idsva_so[{4*n**3}];", "__shared__ T s_a[1588];", "__shared__ T s_b[1268];"] # TODO where did the s_a, s_b memory sizes come from?

    if use_qdd_input:
        shared_mem_vars.insert(-2,"__shared__ T s_qdd[" + str(n) + "]; ")
    self.gen_add_code_lines(shared_mem_vars)
    shared_mem_size = self.gen_idsva_inner_temp_mem_size() if not self.use_dynamic_shared_mem_flag else None
    self.gen_XImats_helpers_temp_shared_memory_code(shared_mem_size)
    if not single_call_timing:
        # load to shared mem and loop over blocks to compute all requested comps
        self.gen_add_parallel_loop("k","NUM_TIMESTEPS",use_thread_group,block_level = True)
        if use_qdd_input:
            self.gen_kernel_load_inputs("q_qd","stride_q_qd",str(n + NUM_POS),use_thread_group,"qdd",str(n),str(n))
        else:
            self.gen_kernel_load_inputs("q_qd","stride_q_qd",str(n + NUM_POS),use_thread_group)
        # compute
        self.gen_add_code_line("// compute")
        self.gen_load_update_XImats_helpers_function_call(use_thread_group)
        self.gen_idsva_so_inner_function_call(use_thread_group)
        self.gen_add_sync(use_thread_group)
        # save to global
        self.gen_kernel_save_result("idsva_so",str(4*n**3),str(4*n**3),use_thread_group)
        self.gen_add_end_control_flow()
    else:
        #repurpose NUM_TIMESTEPS for number of timing reps
        if use_qdd_input:
            self.gen_kernel_load_inputs_single_timing("q_qd",str(2*n),use_thread_group,"qdd",str(n))
        else:
            self.gen_kernel_load_inputs_single_timing("q_qd",str(2*n),use_thread_group)
        # then compute in loop for timing
        self.gen_add_code_line("// compute with NUM_TIMESTEPS as NUM_REPS for timing")
        self.gen_add_code_line("for (int rep = 0; rep < NUM_TIMESTEPS; rep++){", True)
        self.gen_load_update_XImats_helpers_function_call(use_thread_group)
        self.gen_idsva_so_inner_function_call(use_thread_group)
        self.gen_add_end_control_flow()
        # save to global
        self.gen_kernel_save_result_single_timing("idsva_so",str(4*n**3),use_thread_group)
    self.gen_add_end_function()

def gen_idsva_so_host(self, mode = 0):
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
    func_def_start = "void idsva_so_host(gridData<T> *hd_data, const robotModel<T> *d_robotModel, const T gravity, const int num_timesteps,"
    func_def_end =   "                      const dim3 block_dimms, const dim3 thread_dimms, cudaStream_t *streams) {"
    if single_call_timing:
        func_def_start = func_def_start.replace("(", "_single_timing(")
        func_def_end = "              " + func_def_end
    if compute_only:
        func_def_start = func_def_start.replace("(", "_compute_only(")
        func_def_end = "             " + func_def_end.replace(", cudaStream_t *streams", "")
    # then generate the code
    self.gen_add_func_doc("Compute IDSVA-SO (Inverse Dynamics - Spatial Vector Algebra - Second Order)",\
                          func_notes,func_params,None)
    self.gen_add_code_line("template <typename T, bool USE_QDD_FLAG = false, bool USE_COMPRESSED_MEM = false>")
    self.gen_add_code_line("__host__")
    self.gen_add_code_line(func_def_start)
    self.gen_add_code_line(func_def_end, True)
    func_call_start = "idsva_so_kernel<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->idsva_so, \
        hd_data->d_q_qd,stride_q_qd,"
    func_call_end = "d_robotModel,gravity,num_timesteps);"
    if single_call_timing:
        func_call_start = func_call_start.replace("kernel<T>","kernel_single_timing<T>")

# TODO
    if not compute_only:
        # start code with memory transfer
        self.gen_add_code_lines(["// start code with memory transfer", \
                                 "int stride_q_qd;", \
                                 "if (USE_COMPRESSED_MEM) {stride_q_qd = 2*NUM_JOINTS; " + \
                                    "gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd,hd_data->h_q_qd,stride_q_qd*" + \
                                    ("num_timesteps*" if not single_call_timing else "") + "sizeof(T),cudaMemcpyHostToDevice,streams[0]));}", \
                                 "else {stride_q_qd = 3*NUM_JOINTS; " + \
                                    "gpuErrchk(cudaMemcpyAsync(hd_data->d_q_qd_u,hd_data->h_q_qd_u,stride_q_qd*" + \
                                    ("num_timesteps*" if not single_call_timing else "") + "sizeof(T),cudaMemcpyHostToDevice,streams[0]));}", \
                                 "if (USE_QDD_FLAG) {gpuErrchk(cudaMemcpyAsync(hd_data->d_qdd,hd_data->h_qdd,NUM_JOINTS*" + \
                                    ("num_timesteps*" if not single_call_timing else "") + "sizeof(T),cudaMemcpyHostToDevice,streams[1]));}", \
                                 "gpuErrchk(cudaDeviceSynchronize());"])
    else:
        self.gen_add_code_line("int stride_q_qd = USE_COMPRESSED_MEM ? 2*NUM_JOINTS: 3*NUM_JOINTS;")
    # then compute but adjust for compressed mem and qdd usage
    self.gen_add_code_line("// then call the kernel")
    func_call = func_call_start + func_call_end
    func_call_with_qdd = func_call_start + "hd_data->d_qdd, " + func_call_end
    # add in compressed mem adjusts
    func_call_mem_adjust = "    if (USE_COMPRESSED_MEM) {" + func_call + "}"
    func_call_mem_adjust2 = "    else                    {" + func_call.replace("hd_data->d_q_qd","hd_data->d_q_qd_u") + "}"
    func_call_with_qdd_mem_adjust = "    if (USE_COMPRESSED_MEM) {" + func_call_with_qdd + "}"
    func_call_with_qdd_mem_adjust2 = "    else                    {" + func_call_with_qdd.replace("hd_data->d_q_qd","hd_data->d_q_qd_u") + "}"
    # compule into a set of code
    func_call_code = ["if (USE_QDD_FLAG) {", func_call_with_qdd_mem_adjust, func_call_with_qdd_mem_adjust2, "}", \
                      "else {", func_call_mem_adjust, func_call_mem_adjust2, "}", "gpuErrchk(cudaDeviceSynchronize());"]
    # wrap function call in timing (if needed)
    if single_call_timing:
        func_call_code.insert(0,"struct timespec start, end; clock_gettime(CLOCK_MONOTONIC,&start);")
        func_call_code.append("clock_gettime(CLOCK_MONOTONIC,&end);")
    self.gen_add_code_lines(func_call_code)
    if not compute_only:
        # then transfer memory back
        self.gen_add_code_lines(["// finally transfer the result back", \
                                 "gpuErrchk(cudaMemcpy(hd_data->h_idsva_so,hd_data->d_idsva_so,4*NUM_JOINTS*NUM_JOINTS*NUM_JOINTS*" + \
                                    ("num_timesteps*" if not single_call_timing else "") + "sizeof(T),cudaMemcpyDeviceToHost));",
                                 "gpuErrchk(cudaDeviceSynchronize());"])
    # finally report out timing if requested
    if single_call_timing:
        self.gen_add_code_line("printf(\"Single Call ID %fus\\n\",time_delta_us_timespec(start,end)/static_cast<double>(num_timesteps));")
    self.gen_add_end_function()

def gen_idsva_so(self, use_thread_group = False):
    # gen the inner code
    self.gen_idsva_so_inner(use_thread_group)
    # gen the wrapper code for with and without qdd
    self.gen_idsva_so_device(use_thread_group,True)
    self.gen_idsva_so_device(use_thread_group,False)
    # and the kernels
    self.gen_idsva_so_kernel(use_thread_group,True,True)
    self.gen_idsva_so_kernel(use_thread_group,True,False)
    self.gen_idsva_so_kernel(use_thread_group,False,True)
    self.gen_idsva_so_kernel(use_thread_group,False,False)
    # and host wrapeprs
    self.gen_idsva_so_host(0)
    self.gen_idsva_so_host(1)
    self.gen_idsva_so_host(2)