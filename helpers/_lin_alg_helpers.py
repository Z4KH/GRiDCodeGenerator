def gen_invert_matrix(self, use_thread_group=False):
    """
    This function generates a matrix inversion function for cuda.
    The function employs Gaussian elimination.
    """

    self.gen_add_func_doc("Compute the inverse of a matrix", ["Uses gaussian elimination"], \
                          ['dimA is number of rows in A', \
                           'A is a pointer to the original invertible matrix. It is turned into an identity matrix', \
                           'Ainv is a pointer to an identity matrix that will be transformed into the inverse of A', \
                            's_temp is a pointer to temporary memory of size 4*dimA'])
    self.gen_add_code_line("template <typename T>")
    self.gen_add_code_line("__device__")
    self.gen_add_code_line("void invert_matrix(uint32_t dimA, T *A, T *Ainv, T *s_temp) {", True)
    self.gen_add_code_line("for (unsigned pivRC = 0; pivRC < dimA; pivRC++) {", True)   # iterate over diagonal
    self.gen_add_code_line("unsigned pivColOffset = pivRC*dimA;")                       # offset to first value of pivot row
    self.gen_add_code_line("T pvInv = static_cast<T>(1)/A[pivRC + pivColOffset];")      # 1/pivot

    # save the pivot row and column values
    self.gen_add_parallel_loop("ind", "dimA", use_thread_group)
    self.gen_add_code_line("s_temp[ind] = static_cast<T>(A[pivRC * dimA + ind]);")
    self.gen_add_code_line("s_temp[ind+dimA] = static_cast<T>(Ainv[pivRC * dimA + ind]);")
    self.gen_add_code_line("s_temp[ind+dimA*2] = static_cast<T>(A[pivRC + dimA * ind]);")
    self.gen_add_code_line("s_temp[ind+dimA*3] = static_cast<T>(Ainv[pivRC + dimA * ind]);")
    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)

    # run gaussian elimation for the pivot row and column
    self.gen_add_parallel_loop("ind", "dimA*dimA", use_thread_group)
    self.gen_add_code_line("unsigned row = ind % dimA, col = ind / dimA;")
    # apply to the pivot row
    self.gen_add_code_line("if (row == pivRC) {", True)
    self.gen_add_code_line("A[row * dimA + col] *= pvInv;") # put 1 on the diagonal by multiplying row by inverse
    self.gen_add_code_line("Ainv[row * dimA + col] *= pvInv;")
    self.gen_add_end_control_flow()
    # apply to other rows by reducing entries on the pivot column to 0s
    self.gen_add_code_line("else {", True)
    self.gen_add_code_line("T multiplier = s_temp[row+dimA*2] / s_temp[pivRC];")
    self.gen_add_code_line("A[row * dimA + col] -= multiplier * s_temp[col];")
    self.gen_add_code_line("Ainv[row * dimA + col] -= multiplier * s_temp[col+dimA];")
    self.gen_add_end_control_flow()

    self.gen_add_end_control_flow()
    self.gen_add_sync(use_thread_group)
    self.gen_add_end_control_flow()
    self.gen_add_end_function()
    return