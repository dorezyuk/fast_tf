#!/usr/bin/env python3

# Plots the data obtained on a "AMD Ryzen 5 PRO 4650U with Radeon Graphics" CPU.
# To reproduce the results run the publisher.py node in one terminal.
# Run then the listener_new or listener_legacy node in another terminal. Set the
# number of threads via the "/threads" parameter and note down the result in the
# according CSV file.
#
# You can use the "ps  -p $(pgrep -d',' -f listener)  -o %cpu | tail -n +2"
# command to get the CPU usage of the running listener node.
#
# The script must be run from the same directory where the listener_<>_stats.csv
# files are.

import pandas as pd
import matplotlib.pyplot as plt

if __name__ == "__main__":
    try:
        df_leg = pd.read_csv("listener_legacy_stats.csv")
        df_new = pd.read_csv("listener_new_stats.csv")
    except FileExistsError as ex:
        print("Failed to open the file: {}".format(ex))
        exit(-1)

    fig = plt.figure()
    ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])

    ax.plot(df_leg.cpu, label="tf2_ros::Buffer")
    ax.plot(df_new.cpu, label="fast_tf::Buffer")
    ax.set_xticks([0, 1, 2, 3])
    ax.set_xticklabels(["10", "50", "100", "200"])
    ax.set_xlabel("number of query threads")
    ax.set_ylabel("cpu in %")
    ax.legend()
    ax.grid()
    ax.autoscale(tight=True, axis="x")
    plt.savefig("listener_stats.png")
