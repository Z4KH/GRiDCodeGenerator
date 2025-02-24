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
        s_mem_name = "s_mem", \
        gravity_name = "gravity"
    )
    if updated_var_names is not None:
        for key,value in updated_var_names.items():
            var_names[key] = value
    id_so_code_start = "idsva_so_inner<T>(" + var_names["s_idsva_so_name"] + ", " + var_names["s_q_name"] + ", " + var_names["s_qd_name"] + ", " + var_names["s_qdd_name"] + ", "
    id_so_code_middle = self.gen_insert_helpers_function_call()
    id_so_code_end = f'' + var_names["s_mem_name"] + ", " + var_names["gravity_name"] + ");"
    if use_thread_group:
        id_so_code_start = id_so_code_start.replace("(","(tgrp, ")
    id_so_code = id_so_code_start + id_so_code_middle + id_so_code_end
    self.gen_add_code_line(id_so_code)

def gen_idsva_so_inner(self, use_thread_group = False, use_qdd_input = False):
    """
    Generates the inner device function to compute the second order
    idsva.
    """
    # TODO - branching
    NV = self.robot.get_num_vel()

    # construct the boilerplate and function definition
    func_params = ["s_idsva_so is a pointer to memory for the final result of size 4*NUM_JOINTS*NUM_JOINTS*NUM_JOINTS = " + str(4*NV**3), \
                   "s_q is the vector of joint positions", \
                   "s_qd is the vector of joint velocities", \
                   "s_qdd is the vector of joint accelerations", \
    # TODO - shared memory size
                   "s_mem is a pointer to helper shared memory of size  = " + \
                            str(self.gen_idsva_so_inner_temp_mem_size()), \
                   "gravity is the gravity constant"]
    func_def_start = "void idsva_so_inner(T *s_idsva_so, const T *s_q, const T *s_qd, T *s_qdd, "
    func_def_end = "T *s_mem, const T gravity) {"
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


    # MEMORY LAYOUT (s_mem):
        # Xup(36 * NJ)/Xdown(36*NJ)/
            # vJ(6 * NJ)/f(6*NJ)/ICT_S(6*NJ) & 
            # v(6 * NJ)/psid+Sd(6*NJ) & 
            # Sd(6 * NJ) & 
            # aJ(6 * NJ)/IC_v (6 * NJ)/IC_S (6*NJ)/T1 (6*NJ) & 
            # a(6 * NJ) & 
            # psid(6 * NJ)
        # IC (36 * NJ)
        # I_Xup (36 * NJ)/
            # S (6*NJ) & 
            # psidd (6 * NJ) & 
            # a_world (6)
            # T2 (6*NJ)
            # T3 (6*NJ)
            # T4 (6*NJ)
        # BC (36 * NJ)
        # B_IC_S (36*NJ)/D3 (36*NJ)
        # B_IC_psid (36*NJ)
        # crm_v (36 * NJ)/crm_S (36*NJ)
        # crf_v (36 * NJ)/crf_S (36*NJ)
        # crm_psid (36 * NJ)
        # crf_psid (36 * NJ)
        # IC_psid (6 * NJ)
        # icrf_f (36 * NJ)/D1 (36*NJ)
        # D2 (36*NJ)
        # D4 (36*NJ)
        # t - t1/t2/t3/t4/t5/t6/t7/t8/t9 [(\sum_{i=1}^NJ i) * 36]

    vars = [
        '// Relevant Tensors in the order they appear',
        'T *I = s_XImats + XIMAT_SIZE*NUM_JOINTS;', # Inertia Matrices (6x6 for each joint)
        'T *Xup = s_mem;', # Spatial Transforms from parent to child (6x6 for each joint)
        'T *IC = Xup + XIMAT_SIZE*NUM_JOINTS;', # Centroidal Inertia (6x6 for each joint)
        # Xup, I_Xup done being used
        'T *Xdown = Xup;\n', # Spatial Transforms from child to parent (6x6 for each joint)
        'T *S = IC + XIMAT_SIZE*NUM_JOINTS;', # Transformed Joint Subspace Tensors (6x1 for each joint)
        # Xdown done being used
        'T *vJ = Xdown;', # Non-propogated Joint Spatial velocities (6x1 for each joint),
        'T *v = vJ + 6*NUM_JOINTS;', # Joint Spatial velocities (6x1 for each joint),
        'T *Sd = v + 6*NUM_JOINTS;', # Time derivative of Joint subspace tensor due to each joint moving (6x1 for each joint),
        'T *aJ = Sd + 6*NUM_JOINTS;', # Non-propogated Joint Spatial accelerations (6x1 for each joint),
        'T *a = aJ + 6*NUM_JOINTS;', # Joint Spatial accelerations (6x1 for each joint),
        'T *psid = a + 6*NUM_JOINTS;', # Time derivative of joint subspace tensor due to each joint's parent moving (6x1 for each joint),
        'T *psidd = S + 6*NUM_JOINTS;', # 2nd Time derivative of joint subspace tensor due to each joint's parent moving (6x1 for each joint),
        'T *a_world = psidd + 6*NUM_JOINTS;', # Acceleration of the world frame (6x1)',
        'T *BC = S + 36*NUM_JOINTS;', # Composite body-Coriolis Bias tensor (6x6 for each joint)',
        'T *f = vJ;' # Joint Spatial forces (6x1 for each joint),
        'T *B_IC_S = BC + 36*NUM_JOINTS;', # Body coriolis tensor wrt joint subspace (6x6 for each joint)',
        'T *B_IC_psid = B_IC_S + 36*NUM_JOINTS;', # Body coriolis tensor wrt psid (6x6 for each joint)',

        '\n\n',
        '// Temporary Variables for Computations',
        'T *I_Xup = S;', # Temporary to compute IC for I * Xup (6x6 for each joint)
        'T *crm_v = B_IC_psid + 36*NUM_JOINTS;', # Motion cross product of v (6x6 for each joint)
        'T *crf_v = crm_v + 36*NUM_JOINTS;', # Force cross product of v (6x6 for each joint)',
        'T *IC_v = aJ;', # IC @ v (6x1 for each joint)',
        'T *crm_S = crm_v;' # Motion cross product of S (6x6 for each joint),
        'T *crf_S = crf_v;', # Force cross product of S (6x6 for each joint)',
        'T *IC_S = IC_v;' # IC @ S (6x1 for each joint)',
        'T *crm_psid = crf_v + 36*NUM_JOINTS;', # Motion cross product of psid (6x6 for each joint)',
        'T *crf_psid = crm_psid + 36*NUM_JOINTS;', # Force cross product of psid (6x6 for each joint)',
        'T *IC_psid = crf_psid + 36*NUM_JOINTS;', # IC @ psid (6x6 for each joint)',
        'T *icrf_f = IC_psid + 6*NUM_JOINTS;', # icrf(f) (6x6 for each joint)',
        'T *psid_Sd = v;', # psid + Sd (6x1 for each joint)',
        'T *ICT_S = f;', # IC @ S (6x1 for each joint)',

        '\n\n',
        '// Main Temporary Tensors For Backward Pass',
        'T *T1 = IC_S;', # Temporary for IC @ S (6x1 for each joint)',
        'T *T2 = a_world + 6;', # Temporary for -BC.T @ S (6x1 for each joint)',
        'T *T3 = T2 + 6*NUM_JOINTS;', # Temporary matrix (6x1 for each joint)',
        'T *T4 = T3 + 6*NUM_JOINTS;', # Temporary matrix (6x1 for each joint)',
        'T *D1 = icrf_f;', # Temporary D1 tensor (6x6 for each joint)',
        'T *D2 = D1 + 36*NUM_JOINTS;', # Temporary D2 tensor (6x6 for each joint)',
        'T *D3 = B_IC_S;', # Temporary D3 tensor - same as B(IC, S) (6x6 for each joint)',
        'T *D4 = D2 + 36*NUM_JOINTS;', # Temporary D4 tensor (6x6 for each joint)',
        'T *t = D4 + 36*NUM_JOINTS;', # Temporary outer product tensor for t1-t9 (6x6 for each joint)',

        '\n\n',
        '// Final Tensors for Output',
        'T *d2tau_dq2 = s_idsva_so;' # Second positional derivative of the joint torques (NJxNJXNJ)',
        'T *d2tau_dqd2 = d2tau_dq2+ NUM_JOINTS*NUM_JOINTS*NUM_JOINTS;', # Second velocity derivative of the joint torques (NJxNJXNJ)',
        'T *d2tau_cross = d2tau_dqd2 + NUM_JOINTS*NUM_JOINTS*NUM_JOINTS;', # Cross position/velocity derivative of the joint torques (NJxNJXNJ)',
        'T *dM_dq = d2tau_cross + NUM_JOINTS*NUM_JOINTS*NUM_JOINTS;', # Positional Derivative of the mass matrix (NJxNJXNJ)',
    ]
    
    self.gen_add_code_lines(vars)


    # Compute Xup transformations
    self.gen_add_code_line("\n")
    self.gen_add_code_line("// Compute Xup - parent to child transformation matrices")
    # Copy X[0] to Xup[0] - X matrices always 6x6
    self.gen_add_parallel_loop('i','XIMAT_SIZE',use_thread_group) 
    self.gen_add_code_line('Xup[i] = s_XImats[i];')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)
    # Next, loop over remaining joints
    self.gen_add_code_line('#pragma unroll')
    self.gen_add_code_line('for (int joint = 1; joint < NUM_JOINTS; ++joint) {', 1)
    self.gen_add_code_line('// Compute Xup[joint]')
    self.gen_add_code_line('int X_idx = joint*XIMAT_SIZE;')
    self.gen_add_parallel_loop('i','XIMAT_SIZE',use_thread_group)
    self.gen_add_code_line('matmul<T>(i, &Xup[X_idx - XIMAT_SIZE], &s_XImats[X_idx], &Xup[X_idx], XIMAT_SIZE, 0);')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)
    self.gen_add_end_control_flow()

    # Next compute IC - Centroidal Rigid Body Inertia
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line("// Compute IC - Centroidal Rigid Body Inertia")
    # First I @ Xup
    self.gen_add_code_line('// First I @ Xup')
    self.gen_add_parallel_loop('i','XIMAT_SIZE*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('// All involved matrices are 6x6')
    self.gen_add_code_line('matmul<T>(i, Xup, I, I_Xup, 36, false);')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)
    # Next Xup.T @ I
    self.gen_add_code_line('// Next Xup.T @ I')
    self.gen_add_parallel_loop('i','XIMAT_SIZE*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('// All involved matrices are 6x6')
    self.gen_add_code_line('int mat_idx = (i / 36) * 36;')
    self.gen_add_code_line("matmul_trans<T>(i % 36, &Xup[mat_idx], &I_Xup[mat_idx], &IC[mat_idx], 'a');")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Next compute Xdown transformations
    # Just the transpose of internal 3x3 submatrices
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line("// Compute Xdown - child to parent transformation matrices")
    self.gen_add_parallel_loop('i','XIMAT_SIZE*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('T temp = Xup[i]; // Xup and Xdown are in the same memory')
    self.gen_add_code_line('int sub_idx = i % XIMAT_SIZE;')
    # TODO fix magic numbers
    self.gen_add_code_line('if (sub_idx % 18 == 1 || sub_idx % 18 == 4 || sub_idx % 18 == 8 || sub_idx % 18 == 11) {', True)
    self.gen_add_code_line(f'Xdown[i] = Xup[i+5];')
    self.gen_add_code_line(f'Xdown[i+5] = temp;')
    self.gen_add_end_control_flow()
    self.gen_add_code_line('else if (sub_idx % 18 == 2 || sub_idx % 18 == 5) {', True)
    self.gen_add_code_line(f'Xdown[i] = Xup[i+10];')
    self.gen_add_code_line('Xdown[i+10] = temp;')
    self.gen_add_end_control_flow()
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Transform S
    parent_ind_cpp, S_ind_cpp = self.gen_topology_helpers_pointers_for_cpp([i for i in range(NV)], NO_GRAD_FLAG = True)
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Transform S')
    self.gen_add_parallel_loop('i','6*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('int joint = i / 6;')
    self.gen_add_code_line(f'S[i] = Xdown[joint*XIMAT_SIZE + {S_ind_cpp}*6 + (i % 6)];')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Compute vJ = S @ qd & aJ = S @ qdd in parallel
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Compute vJ = S @ qd & aJ = S @ qdd')
    self.gen_add_parallel_loop('i','2*6*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('int joint = i / 6;')
    self.gen_add_code_line('if (joint < NUM_JOINTS) vJ[i] = S[i] * s_qd[joint];')
    self.gen_add_code_line('else aJ[i - 6*NUM_JOINTS] = S[i - 6*NUM_JOINTS] * s_qdd[joint - NUM_JOINTS];')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Compute v = v[parent] + vJ
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Compute v = v[parent] + vJ')
    self.gen_add_code_line('#pragma unroll')
    self.gen_add_code_line('for (int jid = 0; jid < NUM_JOINTS; ++jid) {', 1)
    self.gen_add_parallel_loop('i','6',use_thread_group)
    self.gen_add_code_line('if (jid == 0) v[i] = vJ[i];')
    self.gen_add_code_line(f'else v[jid*6 + i] = v[{parent_ind_cpp}*6 + i] + vJ[jid*6 + i];')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)
    self.gen_add_end_control_flow()

    # Finish aJ += crm(v[parent])@vJ
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Finish aJ += crm(v[parent])@vJ')
    self.gen_add_code_line('// For base, v[parent] = 0')
    self.gen_add_parallel_loop('i','6*(NUM_JOINTS-1)',use_thread_group)
    self.gen_add_code_line('int jid = 1 + (i / 6); // Skip base joint')
    self.gen_add_code_line('int index = i % 6;')
    self.gen_add_code_line(f'aJ[6+i] += crm_mul<T>(index, &v[{parent_ind_cpp}*6], &vJ[jid*6]); // Skip Base Joint')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Compute Sd = crm(v) @ S & psid = crm(v[parent]) @ S
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Compute Sd = crm(v) @ S & psid = crm(v[parent]) @ S')
    self.gen_add_code_line('// For base, v[parent] = 0')
    self.gen_add_parallel_loop('i','2*6*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('int jid = (i / 6) % NUM_JOINTS;')
    self.gen_add_code_line('int index = i % 6;')
    self.gen_add_code_line('if (i < 6*NUM_JOINTS) Sd[i] = crm_mul<T>(index, &v[jid*6], &S[jid*6]);')
    self.gen_add_code_line('else {', True)
    self.gen_add_code_line('if (jid == 0) psid[index] = 0;')
    self.gen_add_code_line(f'else psid[i - 6 * NUM_JOINTS] = crm_mul<T>(index, &v[{parent_ind_cpp}*6], &S[jid*6]);')   
    self.gen_add_end_control_flow()
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Compute a = a[parent] + aJ
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Compute a = a[parent] + aJ')
    self.gen_add_code_line('#pragma unroll')
    self.gen_add_code_line('for (int jid = 0; jid < NUM_JOINTS; ++jid) {', 1)
    self.gen_add_parallel_loop('i','6',use_thread_group)
    self.gen_add_code_line("if (jid == 0) a[i] = aJ[i] + s_XImats[5*6 + i] * gravity; // Base joint's parent is the world")
    self.gen_add_code_line(f'else a[jid*6 + i] = a[{parent_ind_cpp}*6 + i] + aJ[jid*6 + i];')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)
    self.gen_add_end_control_flow()

    # Initialize a_world
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Initialize a_world')
    self.gen_add_parallel_loop('i','6',use_thread_group)
    self.gen_add_code_line('if (i < 5) a_world[i] = 0;')
    self.gen_add_code_line('else a_world[5] = gravity;')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)
    
    # Compute psidd = crm(a[parent])@S + crm(v[parent])@psid & IC_v
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Compute psidd = crm(a[parent])@S + crm(v[:,i])@psid[:,i] & IC @ v (for BC) in parallel')
    self.gen_add_parallel_loop('i','2*6*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('int jid = (i / 6) % NUM_JOINTS;')
    self.gen_add_code_line('int index = i % 6;')
    self.gen_add_code_line('if (i < 6*NUM_JOINTS) {', True)
    self.gen_add_code_line('if (jid == 0) psidd[i] = crm_mul<T>(index, a_world, S) + crm_mul<T>(index, v, psid);')
    self.gen_add_code_line(f'else psidd[i] = crm_mul<T>(index, &a[{parent_ind_cpp}*6], &S[jid*6]) + crm_mul<T>(index, &v[{parent_ind_cpp}*6], &psid[jid*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_code_line(f'else IC_v[i - 6*NUM_JOINTS] = dot_prod<T, 6, 6, 1>(&IC[index + jid*36], &v[jid*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Begin BC Computation
    # First Compute crm(v) & crf(v)
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Need crm(v), crf(v) for BC computation')
    self.gen_add_parallel_loop('i','2*36*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('int jid = (i / 36) % NUM_JOINTS;')
    self.gen_add_code_line('int col = (i / 6) % 6;')
    self.gen_add_code_line('int row = i % 6;')
    self.gen_add_code_line('if (i < 36*NUM_JOINTS) crm_v[i] = crm<T>(i % 36, &v[jid*6]);')
    self.gen_add_code_line('else crf_v[(jid*36) + row*6 + col] = -crm<T>(i % 36, &v[jid*6]); // crf is negative tranpose of crm')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)


    # Finish BC = crf(v) @ IC + icrf(IC @ v) - IC @ crm(v)
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Finish BC = crf(v) @ IC + icrf(IC @ v) - IC @ crm(v)')
    self.gen_add_parallel_loop('i','36*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('int jid = i / 36;')
    self.gen_add_code_line('int row = i % 6;')
    self.gen_add_code_line('int col_idx = (i / 6) * 6;')
    self.gen_add_code_line('BC[i] = dot_prod<T, 6, 6, 1>(&crf_v[jid*36 + row], &IC[col_idx]) +')
    self.gen_add_code_line('        icrf<T>(i % 36, &IC_v[jid*6]) -')
    self.gen_add_code_line('        dot_prod<T, 6, 6, 1>(&IC[jid*36 + row], &crm_v[col_idx]);')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Next f = IC @ a + crf(v) @ IC @ v
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Compute f = IC @ a + crf(v) @ IC @ v')
    self.gen_add_parallel_loop('i','6*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('int jid = i / 6;')
    self.gen_add_code_line('int row = i % 6;')
    self.gen_add_code_line('f[i] = dot_prod<T, 6, 6, 1>(&IC[jid*36 + row], &a[jid*6]) +')
    self.gen_add_code_line('        dot_prod<T, 6, 6, 1>(&crf_v[jid*36 + row], &IC_v[jid*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Forward Pass Completed
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Forward Pass Completed')
    self.gen_add_code_line('// Now compute the backward pass')

    # Compute IC[parent] += IC[i], BC[parent] += BC[i], f[parent] += f[i]
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Compute IC[parent] += IC[i], BC[parent] += BC[i], f[parent] += f[i]')
    self.gen_add_code_line('#pragma unroll')
    self.gen_add_code_line('for (int jid = NUM_JOINTS-1; jid > 0; --jid) {', 1)
    self.gen_add_parallel_loop('i','36*2 + 6',use_thread_group)
    self.gen_add_code_line(f'if (i < 36) IC[{parent_ind_cpp}*36 + i] += IC[jid*36 + i];')
    self.gen_add_code_line(f'else if (i < 36*2) BC[{parent_ind_cpp}*36 + i - 36] += BC[jid*36 + i - 36];')
    self.gen_add_code_line(f'else f[{parent_ind_cpp}*6 + i - 36*2] += f[jid*6 + i - 36*2];')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)
    self.gen_add_end_control_flow()

    # Begin B(IC, S) & B(IC, psid) computation
    # First compute crm(S), crf(S), IC @ S && crm(psid), crf(psid), IC @ psid, icrf(f), psid+Sd
    self.gen_add_code_line("\n\n")
    self.gen_add_code_line('// Need crm(S), crf(S), IC@S, crm(psid), crf(psid), IC@psid for B computations & icrf(f), psid+Sd for T3,T4')
    self.gen_add_parallel_loop('i','5*36*NUM_JOINTS + 3*6*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('int jid = (i / 36) % NUM_JOINTS;')
    self.gen_add_code_line('int jidMatmul = (i / 6) % NUM_JOINTS;')
    self.gen_add_code_line('int col = (i / 6) % 6;')
    self.gen_add_code_line('int row = i % 6;')
    self.gen_add_code_line('if (i < 36*NUM_JOINTS) crm_S[i] = crm<T>(i % 36, &S[jid*6]);')
    self.gen_add_code_line('else if (i < 2*36*NUM_JOINTS) crf_S[(jid*36) + row*6 + col] = -crm<T>(i % 36, &S[jid*6]); // crf is negative tranpose of crm')
    self.gen_add_code_line('else if (i < 3*36*NUM_JOINTS) crm_psid[jid*36 + col*6 + row] = crm<T>(i % 36, &psid[jid*6]);')
    self.gen_add_code_line('else if (i < 4*36*NUM_JOINTS) crf_psid[(jid*36) + row*6 + col] = -crm<T>(i % 36, &psid[jid*6]); // crf is negative tranpose of crm')
    self.gen_add_code_line('else if (i < 5*36*NUM_JOINTS) icrf_f[i - 4*36*NUM_JOINTS] = icrf<T>(i % 36, &f[jid*6]);')
    self.gen_add_code_line('else if (i < 5*36*NUM_JOINTS + 6*NUM_JOINTS) IC_S[i - 5*36*NUM_JOINTS] = dot_prod<T, 6, 6, 1>(&IC[row + jidMatmul*36], &S[jidMatmul*6]);')
    self.gen_add_code_line('else if (i < 5*36*NUM_JOINTS + 2*6*NUM_JOINTS) psid_Sd[i - 5*36*NUM_JOINTS - 6*NUM_JOINTS] = psid[i - 5*36*NUM_JOINTS - 6*NUM_JOINTS] + Sd[i - 5*36*NUM_JOINTS - 6*NUM_JOINTS];')
    self.gen_add_code_line('else IC_psid[i - 5*36*NUM_JOINTS - 2*6*NUM_JOINTS] = dot_prod<T, 6, 6, 1>(&IC[row + jidMatmul*36], &psid[jidMatmul*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Finish B_IC_S, B_IC_psid
    self.gen_add_code_line('\n\n')
    self.gen_add_code_line('// Finish B_IC_S, B_IC_psid')
    self.gen_add_code_line('// B_IC_S = crf(S) @ IC + icrf(IC @ S) - IC @ crm(S)')
    self.gen_add_code_line('// B_IC_psid = crf(psid) @ IC + icrf(IC @ psid) - IC @ crm(psid)')
    self.gen_add_parallel_loop('i','2*36*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('int jid = (i / 36) % NUM_JOINTS;')
    self.gen_add_code_line('int row = i % 6;')
    self.gen_add_code_line('int col = (i / 6) % 6;')
    self.gen_add_code_line('if (i < 36*NUM_JOINTS) {', True)
    self.gen_add_code_line('B_IC_S[i] = dot_prod<T, 6, 6, 1>(&crf_S[jid*36 + row], &IC[jid*36 + col*6]) + ')
    self.gen_add_code_line('            icrf<T>(i % 36, &IC_S[jid*6]) -') 
    self.gen_add_code_line('            dot_prod<T, 6, 6, 1>(&IC[jid*36 + row], &crm_S[jid*36 + col*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_code_line('else {', True)
    self.gen_add_code_line('B_IC_psid[i - 36*NUM_JOINTS] = dot_prod<T, 6, 6, 1>(&crf_psid[jid*36 + row], &IC[jid*36 + col*6]) + ')
    self.gen_add_code_line('                                icrf<T>(i % 36, &IC_psid[jid*6]) -') 
    self.gen_add_code_line('                                dot_prod<T, 6, 6, 1>(&IC[jid*36 + row], &crm_psid[jid*36 + col*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Compute T2 = -BC.T @ S & T3 = BC @ psid + IC @ psidd + icrf(f) @ S, & T4 = BC @ S + IC @ (psid + Sd), & IC.T @ S for D4
    self.gen_add_code_line('\n\n')
    self.gen_add_code_line('// Compute T2 = -BC.T @ S')
    self.gen_add_code_line('// Compute T3 = BC @ psid + IC @ psidd + icrf(f) @ S')
    self.gen_add_code_line('// Compute T4 = BC @ S + IC @ (psid + Sd)')
    self.gen_add_code_line('// Compute IC.T @ S for D4')
    self.gen_add_parallel_loop('i','4*6*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('int jid = (i / 6) % NUM_JOINTS;')
    self.gen_add_code_line('int row = i % 6;')
    self.gen_add_code_line('if (i < 6*NUM_JOINTS) T2[i] = -dot_prod<T, 6, 1, 1>(&BC[jid*36 + row*6], &S[jid*6]);')
    self.gen_add_code_line('else if (i < 2*6*NUM_JOINTS) {', True)
    self.gen_add_code_line('T3[i - 6*NUM_JOINTS] = dot_prod<T, 6, 6, 1>(&BC[jid*36 + row], &psid[jid*6]) +')
    self.gen_add_code_line('                    dot_prod<T, 6, 6, 1>(&IC[jid*36 + row], &psidd[jid*6]) +')
    self.gen_add_code_line('                    dot_prod<T, 6, 6, 1>(&icrf_f[jid*36 + row], &S[jid*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_code_line('else if (i < 3*6*NUM_JOINTS) {', True)
    self.gen_add_code_line('T4[i - 2*6*NUM_JOINTS] = dot_prod<T, 6, 6, 1>(&BC[jid*36 + row], &S[jid*6]) +')
    self.gen_add_code_line('                    dot_prod<T, 6, 6, 1>(&IC[jid*36 + row], &psid_Sd[jid*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_code_line('else ICT_S[i - 3*6*NUM_JOINTS] = dot_prod<T, 6, 6, 1>(&IC[jid*36 + row], &S[jid*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # Compute D1..D4
    self.gen_add_code_line('\n\n')
    self.gen_add_code_line('// Compute D1, D2, D4')
    self.gen_add_parallel_loop('i','3*36*NUM_JOINTS',use_thread_group)
    self.gen_add_code_line('int jid = (i / 36) % NUM_JOINTS;')
    self.gen_add_code_line('int row = i % 6;')
    self.gen_add_code_line('int col = (i / 6) % 6;')
    self.gen_add_code_line('if (i < 36*NUM_JOINTS) {', True)
    self.gen_add_code_line('D1[i] = dot_prod<T, 6, 6, 1>(&crf_S[jid*36 + row], &IC[jid*36 + col*6]) -')
    self.gen_add_code_line('        dot_prod<T, 6, 6, 1>(&IC[jid*36 + row], &crm_S[jid*36 + col*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_code_line('else if (i < 2*36*NUM_JOINTS) {', True)
    self.gen_add_code_line('D2[i - 36*NUM_JOINTS] = B_IC_psid[i - 36*NUM_JOINTS] +')
    self.gen_add_code_line('                        dot_prod<T, 6, 6, 1>(&crf_S[jid*36 + row], &BC[jid*36 + col*6]) -')
    self.gen_add_code_line('                        dot_prod<T, 6, 6, 1>(&BC[jid*36 + row], &crm_S[jid*36 + col*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_code_line('else D4[i - 2*36*NUM_JOINTS] = icrf<T>(i % 36, &ICT_S[jid*6]);')
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)
    
    
    # Compute t1
    # self.gen_add_code_line('\n\n')
    # self.gen_add_code_line('// Compute t1 = outer(S[j], psid[ancestor])')
    # self.gen_add_code_line('int jids[] = {}; // Joint for each matrix of t1') # TODO fill with joint indexes of each matrix
    # self.gen_add_code_line('int ancestors[] = {}; // Ancestor for each matrix of t1') # TODO fill with ancestor indexes of each matrix
    # self.gen_add_parallel_loop('i',f'{sum([i for i in range(NV+1)])}*36',use_thread_group)
    # self.gen_add_code_line('int jid = jids[i / 36];')
    # self.gen_add_code_line('int ancestor = ancestors[i / 36];')
    # self.gen_add_code_line('t1_outer[mat*36] = outerProduct<T>(&S[jid*6], &psid[ancestor*6] 6, 6, index%36);')



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
                   "d_q_dq_u is the vector of joint positions, velocities, and accelerations", \
                   "stride_q_qd_u is the stide between each q, qd, u", \
                   "d_robotModel is the pointer to the initialized model specific helpers on the GPU (XImats, topology_helpers, etc.)", \
                   "gravity is the gravity constant", \
                   "num_timesteps is the length of the trajectory points we need to compute over (or overloaded as test_iters for timing)"]
    func_notes = []
    func_def_start = "void idsva_so_kernel(T *d_idsva_so, const T *d_q_qd_u, const int stride_q_qd_u, "
    func_def_end = "const robotModel<T> *d_robotModel, const T gravity, const int NUM_TIMESTEPS) {"
    if use_qdd_input: # TODO
        func_def_start += "const T *d_qdd, "
        func_params.insert(-2,"d_qdd is the vector of joint accelerations")
    func_def = func_def_start + func_def_end
    if single_call_timing:
        func_def = func_def.replace("kernel(", "kernel_single_timing(")
    # then generate the code
    self.gen_add_func_doc("Computes the second order derivatives of inverse dynamics",func_notes,func_params,None)
    self.gen_add_code_line("template <typename T>")
    self.gen_add_code_line("__global__")
    self.gen_add_code_line(func_def, True)
    # add shared memory variables
    shared_mem_vars = [f"__shared__ T s_q_qd_u[{n*2+NUM_POS}]; T *s_q = s_q_qd_u; T *s_qd = &s_q_qd_u[{NUM_POS}]; T *s_qdd = &s_q_qd_u[{NUM_POS + n}];", \
                    f"__shared__ T s_idsva_so[{4*n**3}];", f"__shared__ T s_mem[{11040}];"] # TODO where did the s_mem size come from?

    if use_qdd_input:
        shared_mem_vars.insert(-2,"__shared__ T s_qdd[" + str(n) + "]; ")
    self.gen_add_code_lines(shared_mem_vars)
    shared_mem_size = self.gen_idsva_inner_temp_mem_size() if not self.use_dynamic_shared_mem_flag else None
    self.gen_XImats_helpers_temp_shared_memory_code(shared_mem_size)
    if not single_call_timing:
        # load to shared mem and loop over blocks to compute all requested comps
        self.gen_add_parallel_loop("k","NUM_TIMESTEPS",use_thread_group,block_level = True)
        if use_qdd_input: # TODO
            self.gen_kernel_load_inputs("q_qd","stride_q_qd",str(n + NUM_POS),use_thread_group,"qdd",str(n),str(n))
        else:
            self.gen_kernel_load_inputs("q_qd_u","stride_q_qd_u",str(2*n + NUM_POS),use_thread_group)
        # compute
        self.gen_add_code_line("// compute")
        self.gen_load_update_XImats_helpers_function_call(use_thread_group)
        self.gen_idsva_so_inner_function_call(use_thread_group)
        self.gen_add_sync(use_thread_group)
        # save to global TODO - why is this breaking everything
        # self.gen_kernel_save_result("idsva_so",str(4*n**3),str(4*n**3),use_thread_group) # TODO - WHY IS THIS BREAKING EVERYTHING
        self.gen_add_end_control_flow()
    else:
        #repurpose NUM_TIMESTEPS for number of timing reps
        if use_qdd_input: # TODO
            self.gen_kernel_load_inputs_single_timing("q_qd",str(2*n),use_thread_group,"qdd",str(n))
        else:
            self.gen_kernel_load_inputs_single_timing("q_qd_u",str(NUM_POS + 2*n),use_thread_group)
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
    func_call_start = "idsva_so_kernel<T><<<block_dimms,thread_dimms,ID_DYNAMIC_SHARED_MEM_COUNT*sizeof(T)>>>(hd_data->d_idsva_so," + \
        "hd_data->d_q_qd_u,stride_q_qd,"
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
    # TODO - qdd=0 optimization
    # func_call = func_call_start + func_call_end
    # func_call_with_qdd = func_call_start + "hd_data->d_qdd, " + func_call_end
    # # add in compressed mem adjusts
    # func_call_mem_adjust = "    if (USE_COMPRESSED_MEM) {" + func_call + "}"
    # func_call_mem_adjust2 = "    else                    {" + func_call.replace("hd_data->d_q_qd","hd_data->d_q_qd_u") + "}"
    # func_call_with_qdd_mem_adjust = "    if (USE_COMPRESSED_MEM) {" + func_call_with_qdd + "}"
    # func_call_with_qdd_mem_adjust2 = "    else                    {" + func_call_with_qdd.replace("hd_data->d_q_qd","hd_data->d_q_qd_u") + "}"
    # compule into a set of code
    # func_call_code = ["if (USE_QDD_FLAG) {", func_call_with_qdd_mem_adjust, func_call_with_qdd_mem_adjust2, "}", \
    #                   "else {", func_call_mem_adjust, func_call_mem_adjust2, "}", "gpuErrchk(cudaDeviceSynchronize());"]
    
    func_call_code = [f'{func_call_start}{func_call_end}']
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
    # TODO - qdd=0 optimization
    # gen the inner code
    self.gen_idsva_so_inner(use_thread_group)
    # gen the wrapper code for device fn
    self.gen_idsva_so_device(use_thread_group,False)
    # and the kernels
    self.gen_idsva_so_kernel(use_thread_group,False,True)
    self.gen_idsva_so_kernel(use_thread_group,False,False)
    # and host wrapeprs
    self.gen_idsva_so_host(0)
    self.gen_idsva_so_host(1)
    self.gen_idsva_so_host(2)