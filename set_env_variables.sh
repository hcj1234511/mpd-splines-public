# Set conda environment path (use CONDA_PREFIX if available, otherwise use hardcoded path)
if [ -z "$CONDA_PREFIX" ]; then
    CONDA_ENV_PATH=$HOME/miniconda3/envs/mpd-splines-public
else
    CONDA_ENV_PATH=$CONDA_PREFIX
fi

export LD_LIBRARY_PATH=$CONDA_ENV_PATH/lib:$LD_LIBRARY_PATH
export CPATH=$CONDA_ENV_PATH/include:$CPATH
