#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from sshtunnel import SSHTunnelForwarder
import pymongo
import numpy as np
import pandas as pd
import plotly.graph_objects as go



class Config:
    def __init__(self):
        self.dashs = [
            'solid', 'dot', 'dash',
            'longdash', 'dashdot', 'longdashdot',
            'solid', 'dot', 'dash',
            'longdash', 'dashdot', 'longdashdot'
        ]

        self.name_index = {
            "Ascent": 0,
            "Descent": 1,
            "Random ascent": 2,
            "Random descent": 3,
        }

        self.index_name = {v: k for k, v in self.name_index.items()}

        self.colors = [
            'rgb(204, 0, 0)', 'rgb(204, 102, 0)', 'rgb(50, 50, 0)',
            'rgb(0, 204, 0)', 'rgb(0, 102, 204)', 'rgb(204, 0, 204)',
            'rgb(0, 0, 0)', 'rgb(140, 0, 80)', 'rgb(75, 150, 35)',
            'rgb(100, 150, 15)', 'rgb(125, 55, 0)'
        ]
        self.fillcolors = [
            'rgba(68, 40, 40, 0.1)', 'rgba(40, 68, 40, 0.1)', 'rgba(40, 40, 68, 0.1)',
            'rgba(10, 40, 40, 0.1)', 'rgba(40, 10, 40, 0.1)', 'rgba(40, 40, 10, 0.1)',
            'rgba(80, 80, 80, 0.2)', 'rgba(20, 140, 80, 0.2)', 'rgba(80, 130, 10, 0.2)',
            'rgba(125, 0, 80, 0.2)', 'rgba(40, 0, 125, 0.2)'
        ]

config = Config()

def form_drawable_curves(arrays):
    data = np.array(arrays).T
    mean, _min, _max = [], [], []
    for point in data:
        p = np.mean(point)
        mean.append(p)
        std = np.std(point)
        _min.append(p-std)
        _max.append(p + std)
    return mean, _min, _max


def draw(data, corrs):

    for metric, arrays in data.items():
        # init plot
        draw_data = []
        for task, fields in corrs.items():
            ind, length = fields
            to_draw = arrays[ind:ind+length]
            mean, _min, _max = form_drawable_curves(to_draw)
            delta = 1.0 / (len(mean)-1)
            x = [delta * i for i in range(len(mean))]
            # draw curve
            index = config.name_index[task]

            trace = go.Scatter(
                name=task,  # '<b>'+lname+'</b>',
                x=x,
                y=mean,
                mode='lines',
                line=dict(color=config.colors[index], dash=config.dashs[index], width=6),
                # fillcolor=fillcolors[i],
                # fill='tonexty'
            )

            upper_bound = go.Scatter(
                x=x,
                y=_max,
                mode='lines',
                # marker=dict(color="#111"),
                line=dict(width=0),
                showlegend=False,
                fillcolor=config.fillcolors[index],
                fill='tonexty'
            )

            lower_bound = go.Scatter(
                x=x,
                showlegend=False,
                y=_min,
                mode='lines',
                # marker=dict(color="#111"),
                fillcolor=config.fillcolors[index],
                line=dict(width=0)
            )
            draw_data.append(trace)
            draw_data.append(lower_bound)
            draw_data.append(upper_bound)
        # layout
        if metric == "deviation":
            metric_name = "COG deviation, m"
        elif metric == "angular_m":
            metric_name = "Pitch angular vel., rad/s"
        elif metric == "reward":
            metric_name = "Reward"
        else:
            raise NotImplementedError("No metric")
        layout = go.Layout(

            yaxis=dict(
                title=metric_name,  # "'<b>'+y_axis+'</b>',
                # gridcolor='rgba(255,255,255,1.0)'
            ),
            xaxis=dict(
                title="Learning time",  # '<b>'+x_axis+'</b>',
                # range=(range_start, 10000.0)
                # gridcolor='rgba(255,255,255,1.0)'
            ),
            showlegend=True,
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='rgba(0,0,0,0)',
            font=dict(
                family="sans serif",
                size=50,
            ),
            legend=dict(
                # x=0.0,
                # y=1.0,
                bgcolor='white'
            ),
        )

        # show curve

        fig = go.Figure(data=draw_data, layout=layout)
        fig.update_xaxes(showline=True, linewidth=2, linecolor='black',
                         showgrid=True, gridwidth=1, gridcolor='rgba(0,0,150,0.1)')
        fig.update_yaxes(showline=True, linewidth=2, linecolor='black',
                         showgrid=True, gridwidth=1, gridcolor='rgba(0,0,150,0.1)')
        # fig.write_image("images/fig1.eps")
        fig.show()
