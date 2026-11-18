"""Print every group, dataset (shape, dtype) and attribute of one HDF5 episode. No pickle; h5py only. Usage: python -I inspect_h5.py <file.hdf5>"""
import sys, h5py
with h5py.File(sys.argv[1], "r") as f:
    print("root attrs", dict(f.attrs))
    def v(name, o):
        if isinstance(o, h5py.Dataset):
            print(name, o.shape, o.dtype, dict(o.attrs) if o.attrs else "")
        else:
            print(name + "/", dict(o.attrs) if o.attrs else "")
    f.visititems(v)
