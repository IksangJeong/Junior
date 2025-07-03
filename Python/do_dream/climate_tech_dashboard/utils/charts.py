"""
차트 생성 유틸리티 함수들
"""

import plotly.express as px
import plotly.graph_objects as go
import pandas as pd
import numpy as np

# 색상 팔레트
CLIMATE_COLORS = {
    '감축': '#1f77b4',
    '적응': '#ff7f0e', 
    '융복합': '#2ca02c'
}

CHART_COLORS = {
    'primary': '#1f4e79',
    'secondary': '#2e8b57',
    'accent': '#ff6b6b',
    'success': '#4ecdc4',
    'warning': '#f39c12',
    'info': '#667eea'
}

def create_empty_chart(message="데이터가 없습니다"):
    """빈 차트 생성"""
    fig = go.Figure()
    fig.add_annotation(
        text=message,
        xref="paper", yref="paper",
        x=0.5, y=0.5,
        showarrow=False,
        font=dict(size=16, color="gray")
    )
    fig.update_layout(
        xaxis=dict(showgrid=False, showticklabels=False, zeroline=False),
        yaxis=dict(showgrid=False, showticklabels=False, zeroline=False),
        plot_bgcolor='white'
    )
    return fig

def create_pie_chart(data, values_col, names_col, title="", colors=None):
    """통일된 파이차트 생성"""
    if data.empty:
        return create_empty_chart()
    
    fig = px.pie(
        data,
        values=values_col,
        names=names_col,
        title=title,
        color_discrete_sequence=colors or px.colors.qualitative.Set3
    )
    
    fig.update_traces(
        textposition='inside',
        textinfo='percent+label',
        hovertemplate='<b>%{label}</b><br>값: %{value}<br>비율: %{percent}<extra></extra>'
    )
    
    fig.update_layout(
        title_x=0.5,
        font=dict(size=12),
        showlegend=True
    )
    
    return fig

def create_bar_chart(data, x_col, y_col, title="", orientation='v', color_col=None):
    """통일된 막대차트 생성"""
    if data.empty:
        return create_empty_chart()
    
    fig = px.bar(
        data,
        x=x_col if orientation == 'v' else y_col,
        y=y_col if orientation == 'v' else x_col,
        color=color_col,
        title=title,
        orientation='h' if orientation == 'h' else 'v',
        color_discrete_map=CLIMATE_COLORS if color_col == 'field' else None
    )
    
    fig.update_layout(
        title_x=0.5,
        font=dict(size=12)
    )
    
    fig.update_traces(
        hovertemplate='<b>%{x if orientation == "v" else y}</b><br>값: %{y if orientation == "v" else x}<extra></extra>'
    )
    
    return fig

def create_line_chart(data, x_col, y_col, color_col=None, title=""):
    """통일된 라인차트 생성"""
    if data.empty:
        return create_empty_chart()
    
    fig = px.line(
        data,
        x=x_col,
        y=y_col,
        color=color_col,
        title=title,
        markers=True,
        color_discrete_map=CLIMATE_COLORS if color_col == 'field' else None
    )
    
    fig.update_layout(
        title_x=0.5,
        font=dict(size=12)
    )
    
    fig.update_traces(
        line=dict(width=3),
        marker=dict(size=8)
    )
    
    return fig

def create_scatter_plot(data, x_col, y_col, color_col=None, size_col=None, title=""):
    """통일된 산점도 생성"""
    if data.empty:
        return create_empty_chart()
    
    fig = px.scatter(
        data,
        x=x_col,
        y=y_col,
        color=color_col,
        size=size_col,
        title=title,
        color_discrete_map=CLIMATE_COLORS if color_col == 'field' else None
    )
    
    fig.update_layout(
        title_x=0.5,
        font=dict(size=12)
    )
    
    return fig

def create_heatmap(data, x_col, y_col, values_col, title=""):
    """히트맵 생성"""
    if data.empty:
        return create_empty_chart()
    
    # 피벗 테이블 생성
    pivot_data = data.pivot_table(
        values=values_col,
        index=y_col,
        columns=x_col,
        aggfunc='sum',
        fill_value=0
    )
    
    fig = px.imshow(
        pivot_data.values,
        x=pivot_data.columns,
        y=pivot_data.index,
        title=title,
        color_continuous_scale='Blues',
        aspect='auto'
    )
    
    fig.update_layout(
        title_x=0.5,
        font=dict(size=12)
    )
    
    return fig

def create_sunburst_chart(data, path_cols, values_col, title=""):
    """선버스트 차트 생성"""
    if data.empty:
        return create_empty_chart()
    
    fig = px.sunburst(
        data,
        path=path_cols,
        values=values_col,
        title=title
    )
    
    fig.update_layout(
        title_x=0.5,
        font=dict(size=12)
    )
    
    return fig

def create_treemap(data, path_cols, values_col, title=""):
    """트리맵 생성"""
    if data.empty:
        return create_empty_chart()
    
    fig = px.treemap(
        data,
        path=path_cols,
        values=values_col,
        title=title
    )
    
    fig.update_layout(
        title_x=0.5,
        font=dict(size=12)
    )
    
    return fig

def create_gauge_chart(value, title="", max_value=100):
    """게이지 차트 생성"""
    fig = go.Figure(go.Indicator(
        mode="gauge+number+delta",
        value=value,
        domain={'x': [0, 1], 'y': [0, 1]},
        title={'text': title},
        gauge={
            'axis': {'range': [None, max_value]},
            'bar': {'color': CHART_COLORS['primary']},
            'steps': [
                {'range': [0, max_value * 0.5], 'color': "lightgray"},
                {'range': [max_value * 0.5, max_value * 0.8], 'color': "gray"}
            ],
            'threshold': {
                'line': {'color': "red", 'width': 4},
                'thickness': 0.75,
                'value': max_value * 0.9
            }
        }
    ))
    
    fig.update_layout(
        height=300,
        font=dict(size=12)
    )
    
    return fig

def create_correlation_matrix(data, numeric_cols, title="상관관계 매트릭스"):
    """상관관계 매트릭스 생성"""
    if data.empty:
        return create_empty_chart()
    
    # 상관계수 계산
    corr_matrix = data[numeric_cols].corr()
    
    fig = px.imshow(
        corr_matrix,
        title=title,
        color_continuous_scale='RdBu',
        color_continuous_midpoint=0,
        aspect='auto'
    )
    
    fig.update_layout(
        title_x=0.5,
        font=dict(size=12)
    )
    
    return fig

def create_box_plot(data, x_col, y_col, title=""):
    """박스플롯 생성"""
    if data.empty:
        return create_empty_chart()
    
    fig = px.box(
        data,
        x=x_col,
        y=y_col,
        title=title
    )
    
    fig.update_layout(
        title_x=0.5,
        font=dict(size=12)
    )
    
    return fig

def create_histogram(data, x_col, title="", bins=30):
    """히스토그램 생성"""
    if data.empty:
        return create_empty_chart()
    
    fig = px.histogram(
        data,
        x=x_col,
        nbins=bins,
        title=title
    )
    
    fig.update_layout(
        title_x=0.5,
        font=dict(size=12)
    )
    
    return fig

def apply_climate_theme(fig):
    """기후기술 테마 적용"""
    fig.update_layout(
        plot_bgcolor='white',
        paper_bgcolor='white',
        font=dict(family="Arial, sans-serif", size=12, color="#333333"),
        title_font=dict(size=16, color="#1f4e79"),
        colorway=[CHART_COLORS['primary'], CHART_COLORS['secondary'], 
                 CHART_COLORS['accent'], CHART_COLORS['success']],
        margin=dict(l=40, r=40, t=60, b=40)
    )
    
    fig.update_xaxes(
        showgrid=True,
        gridwidth=1,
        gridcolor='lightgray',
        showline=True,
        linewidth=1,
        linecolor='lightgray'
    )
    
    fig.update_yaxes(
        showgrid=True,
        gridwidth=1,
        gridcolor='lightgray',
        showline=True,
        linewidth=1,
        linecolor='lightgray'
    )
    
    return fig

def format_number(value, format_type='comma'):
    """숫자 포맷팅"""
    if pd.isna(value):
        return "N/A"
    
    if format_type == 'comma':
        return f"{value:,.0f}"
    elif format_type == 'percent':
        return f"{value:.1f}%"
    elif format_type == 'decimal':
        return f"{value:.2f}"
    else:
        return str(value)

def create_metric_cards_data(data, metrics):
    """메트릭 카드용 데이터 생성"""
    cards_data = []
    
    for metric in metrics:
        if metric['type'] == 'sum':
            value = data[metric['column']].sum()
        elif metric['type'] == 'mean':
            value = data[metric['column']].mean()
        elif metric['type'] == 'count':
            value = data[metric['column']].count()
        elif metric['type'] == 'nunique':
            value = data[metric['column']].nunique()
        else:
            value = 0
        
        cards_data.append({
            'title': metric['title'],
            'value': format_number(value, metric.get('format', 'comma')),
            'color': metric.get('color', 'primary')
        })
    
    return cards_data