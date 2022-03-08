# Plotting experimental data

We provide two alternatives to plot the data obtained in the experimental phase:

* `pdf` uses `lualatex` and `pgfplots` to generate a PDF with the plots,
* `show` uses `matplotlib` to generate plots for display

The plotting script is written in `python3` and can be used invoking:
```console
foo@bar:~$ python plot.py <1> <2>
```
where `<1>` is either `pdf` or `show` and `<2>` is the (relative or absolute) path to the experimental data file and `python` should point to `python3`.

## How to Use

The folder `example_data` contains a sample trace in the file `example`, obtained in one of our experiments.

To visualize the graphs using `matplotlib`, give the following command:
```console
foo@bar:~$ python plot.py show example_data/example
Reading data from file: example_data/example
* read data with total length: 10000
* figure 1: 3d position
* figure 2: position and velocity (x,y,z)
* figure 3: sensor data and Kalman errors
* figure 4: motor control signals (u1,u2,u3,u4)
```
The script displays the graphs. As noted above, the first figure is a 3d plot of the crazyflie trajectory. 

To get a PDF with the graphs, give the following command:
```console
foo@bar:~$ python python plot.py pdf example_data/example
Reading data from file: example_data/example
* read data with total length: 10000
* saved data to intermediate file: data.csv
* saved tex to intermediate file: example.tex
* compiled latex plot: example.pdf
* removed intermediate file: data.csv
* removed intermediate file: example.tex + .aux .log
```
The script in this case generates a file `example.pdf` which is a one page A4 PDF file with the graphs.
The title of the plot corresponds to the filename (the code in the repository automatically generates filenames without extension as the `example` file).
The only difference between the two sets of graph is that the PDF does not include a 3D plot of the trajectory of the crazyflie (which is best visualized in the interactive setting).

Note: intermediate files (csv, tex, aux, log) are automatically removed.
If you want to keep them, change the value of the variable `remove_intermediate` at line 9 of `plot.py`. 
