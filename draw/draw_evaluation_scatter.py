#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from retriever import retrieve_local
import numpy as np
from plotly.subplots import make_subplots
import plotly.graph_objects as go


def draw(data):
    color = [
        'rgb(204, 0, 0)', 'rgb(204, 102, 0)', 'rgb(50, 50, 0)',
        'rgb(0, 204, 0)', 'rgb(0, 102, 204)', 'rgb(204, 0, 204)',
        'rgb(0, 0, 0)', 'rgb(140, 0, 80)', 'rgb(75, 150, 35)',
        'rgb(100, 150, 15)', 'rgb(125, 55, 0)'
    ]
    smb = {
        0: "cross",
        1: "square",
        2: "x",
        3: "star-triangle-down",
        4: "hourglass",
        5: "circle",
    }
    task_ids = {key: i for i, key in enumerate(data.keys())}
    fig = make_subplots(rows=1, cols=3)

    fig.add_trace(
        go.Scatter(
            x=["Asc-inc-cog", "Asc-uni-cog"],
            y=[
                data["Asc-inc-cog"]["r"]["mean"],
                data["Asc-uni-cog"]["r"]["mean"],
            ],
            mode='markers',
            marker=dict(color=[color[0], color[1]], size=20),
            marker_symbol=[smb[0], smb[1]],
            showlegend=True,
            error_y=dict(
                type='data',  # value of error bar given in data coordinates
                array=[data["Asc-inc-cog"]["r"]["std"], data["Asc-uni-cog"]["r"]["std"]],
                thickness=3,
                visible=True
            ),
        ),
        row=1, col=1
    )

    fig.add_trace(
        go.Scatter(
            x=["Asc-inc-cog", "Asc-uni-cog"],
            y=[
                data["Asc-inc-cog"]["dev"]["mean"],
                data["Asc-uni-cog"]["dev"]["mean"],
            ],
            mode='markers',
            marker=dict(color=[color[0], color[1]], size=20),
            marker_symbol=[smb[0], smb[1]],
            showlegend=True,
            error_y=dict(
                type='data',  # value of error bar given in data coordinates
                array=[data["Asc-inc-cog"]["dev"]["std"], data["Asc-uni-cog"]["dev"]["std"]],
                thickness=3,
                visible=True
            ),
        ),
        row=1, col=2
    )

    fig.add_trace(
        go.Scatter(
            x=["Des-inc-ang", "Des-uni-ang"],
            y=[
                data["Des-inc-ang"]["ang"]["mean"],
                data["Des-uni-ang"]["ang"]["mean"],
            ],
            mode='markers',
            marker=dict(color=[color[2], color[3]], size=20),
            marker_symbol=[smb[2], smb[3]],
            showlegend=True,
            error_y=dict(
                type='data',  # value of error bar given in data coordinates
                array=[data["Des-inc-ang"]["ang"]["std"], data["Des-uni-ang"]["ang"]["std"]],
                thickness=3,
                visible=True
            ),
        ),
        row=1, col=3
    )
    fig.update_layout(
        # title='Absolem platform',
        plot_bgcolor='rgb(255,255,255)',
        showlegend=False,
        width=1700,
        height=600,
        font=dict(
            size=20
        ),
        annotations=[
            go.layout.Annotation(
                text="Task id",
                align='center',
                showarrow=False,
                xref='paper',
                yref='paper',
                x=0.12,
                y=-0.13,
                bordercolor='black',
                borderwidth=0
            ),
            go.layout.Annotation(
                text="(a)",
                align='center',
                showarrow=False,
                xref='paper',
                yref='paper',
                x=0.1325,
                y=-0.19,
                bordercolor='black',
                borderwidth=0
            ),
            go.layout.Annotation(
                text='Task id',
                align='center',
                showarrow=False,
                xref='paper',
                yref='paper',
                x=0.5,
                y=-0.13,
                bordercolor='black',
                borderwidth=0
            ),
            go.layout.Annotation(
                text="(b)",
                align='center',
                showarrow=False,
                xref='paper',
                yref='paper',
                x=0.5,
                y=-0.19,
                bordercolor='black',
                borderwidth=0
            ),
            go.layout.Annotation(
                text='Task id',
                align='center',
                showarrow=False,
                xref='paper',
                yref='paper',
                x=0.88,
                y=-0.13,
                bordercolor='black',
                borderwidth=0
            ),
            go.layout.Annotation(
                text="(c)",
                align='center',
                showarrow=False,
                xref='paper',
                yref='paper',
                x=0.8685,
                y=-0.19,
                bordercolor='black',
                borderwidth=0
            ),
        ]

    )
    fig.update_xaxes(showticklabels=True, showline=True, linewidth=2, linecolor='black',
                     showgrid=True, gridwidth=3, gridcolor='rgba(0,0,0,0.2)', row=1, col=1)
    fig.update_xaxes(showticklabels=True, showline=True, linewidth=2, linecolor='black',
                     showgrid=True, gridwidth=3, gridcolor='rgba(0,0,0,0.2)', row=1, col=2)
    fig.update_xaxes(showticklabels=True, showline=True, linewidth=2, linecolor='black',
                     showgrid=True, gridwidth=3, gridcolor='rgba(0,0,0,0.2)', row=1, col=3)

    fig.update_yaxes(title_text="Progress", showline=True, linewidth=2, linecolor='black',
                     showgrid=True, gridwidth=3, gridcolor='rgba(0,0,0,0.2)', row=1, col=1)
    fig.update_yaxes(title_text="COG deviation, m", showline=True, linewidth=2, linecolor='black',
                     showgrid=True, gridwidth=3, gridcolor='rgba(0,0,0,0.2)', row=1, col=2)
    fig.update_yaxes(title_text="Pitch angular  vel., rad/s", showline=True, linewidth=2, linecolor='black',
                     showgrid=True, gridwidth=3, gridcolor='rgba(0,0,0,0.2)', row=1, col=3)
    fig.show()


def proc(data, corr):
    ret = {}
    for i, exp in enumerate(corr):
        ret[exp] = {
            "r": {
                "mean": np.mean(np.clip(data["reward"][i], 0, 1)),
                "std": np.std(np.clip(data["reward"][i], 0, 1)),
            },
            "dev": {
                "mean": np.mean(data["deviation"][i]),
                "std": np.std(data["deviation"][i]),
            },
            "ang": {
                "mean": np.mean(data["angular_m"][i]),
                "std": np.std(data["angular_m"][i]),
            },
        }
    return ret


if __name__ == "__main__":
    database = "exp_official_eval"
    colls = [
        "asc1_evaluation",
        "des1_evaluation",
        "asc_rand1_evaluation",
        "rand_des1_evaluation",
    ]
    corr = [
        "Asc-inc-cog",
        "Des-inc-ang",
        "Asc-uni-cog",
        "Des-uni-ang",
    ]
    fields = ["reward", "deviation", "angular_m"]  #
    raw_data = retrieve_local(database, colls, fields)
    proc_data = proc(raw_data, corr)
    draw(proc_data)

