import { useState } from 'react';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from 'recharts';
import type { TimeSeriesData } from '../types/sopot';

// Aerospace theme colors (matching CSS variables in responsive.css)
const THEME_COLORS = {
  cyan: '#00d4ff',    // --accent-cyan
  green: '#00ff88',   // --accent-green
  amber: '#ffaa00',   // --accent-amber
  red: '#ff3b3b',     // --accent-red
};

interface PlotPanelProps {
  timeSeries: TimeSeriesData | null;
}

type PlotType =
  | 'altitude'
  | 'speed'
  | 'velocity'
  | 'acceleration'
  | 'mass'
  | 'forces';

interface ChartDataPoint {
  time: number;
  [key: string]: number;
}

interface PlotConfig {
  id: PlotType;
  title: string;
  yLabel: string;
  lines: Array<{
    dataKey: string;
    name: string;
    color: string;
    extractData: (data: TimeSeriesData) => number[];
  }>;
}

const PLOT_CONFIGS: PlotConfig[] = [
  {
    id: 'altitude',
    title: 'Altitude vs Time',
    yLabel: 'Altitude (m)',
    lines: [
      {
        dataKey: 'altitude',
        name: 'Altitude',
        color: THEME_COLORS.cyan,
        extractData: (data) => data.kinematics.altitude,
      },
    ],
  },
  {
    id: 'speed',
    title: 'Speed vs Time',
    yLabel: 'Speed (m/s)',
    lines: [
      {
        dataKey: 'speed',
        name: 'Speed',
        color: THEME_COLORS.green,
        extractData: (data) => data.kinematics.speed,
      },
    ],
  },
  {
    id: 'velocity',
    title: 'Velocity Components (ENU Frame)',
    yLabel: 'Velocity (m/s)',
    lines: [
      {
        dataKey: 'vel_x',
        name: 'East (Vx)',
        color: THEME_COLORS.red,
        extractData: (data) => data.kinematics.vel_x,
      },
      {
        dataKey: 'vel_y',
        name: 'North (Vy)',
        color: THEME_COLORS.cyan,
        extractData: (data) => data.kinematics.vel_y,
      },
      {
        dataKey: 'vel_z',
        name: 'Up (Vz)',
        color: THEME_COLORS.green,
        extractData: (data) => data.kinematics.vel_z,
      },
    ],
  },
  {
    id: 'acceleration',
    title: 'Acceleration Components (ENU Frame)',
    yLabel: 'Acceleration (m/s²)',
    lines: [
      {
        dataKey: 'accel_x',
        name: 'East (Ax)',
        color: THEME_COLORS.red,
        extractData: (data) => data.dynamics.accel_x,
      },
      {
        dataKey: 'accel_y',
        name: 'North (Ay)',
        color: THEME_COLORS.cyan,
        extractData: (data) => data.dynamics.accel_y,
      },
      {
        dataKey: 'accel_z',
        name: 'Up (Az)',
        color: THEME_COLORS.green,
        extractData: (data) => data.dynamics.accel_z,
      },
    ],
  },
  {
    id: 'mass',
    title: 'Mass vs Time',
    yLabel: 'Mass (kg)',
    lines: [
      {
        dataKey: 'mass',
        name: 'Mass',
        color: THEME_COLORS.amber,
        extractData: (data) => data.dynamics.mass,
      },
    ],
  },
  {
    id: 'forces',
    title: 'Force Magnitudes',
    yLabel: 'Thrust (N) and Gravity (m/s²)',
    lines: [
      {
        dataKey: 'thrust',
        name: 'Thrust (N)',
        color: THEME_COLORS.red,
        extractData: (data) => data.forces.thrust,
      },
      {
        dataKey: 'gravity',
        name: 'Gravity (m/s²)',
        color: THEME_COLORS.cyan,
        extractData: (data) => data.forces.gravity,
      },
    ],
  },
];

function formatChartData(timeSeries: TimeSeriesData, config: PlotConfig): ChartDataPoint[] {
  const length = timeSeries.time.length;

  // Validate all data arrays have consistent lengths
  const allArrays = [
    timeSeries.time,
    ...config.lines.map(line => line.extractData(timeSeries))
  ];

  const hasConsistentLength = allArrays.every(arr => arr.length === length);
  if (!hasConsistentLength) {
    console.warn('Inconsistent data array lengths detected');
  }

  // Pre-extract all data arrays outside the loop for performance
  const extractedData = config.lines.map((line) => ({
    dataKey: line.dataKey,
    values: line.extractData(timeSeries),
  }));

  const chartData: ChartDataPoint[] = [];

  for (let i = 0; i < length; i++) {
    const point: ChartDataPoint = { time: timeSeries.time[i] };

    extractedData.forEach(({ dataKey, values }) => {
      // Handle missing data gracefully
      point[dataKey] = i < values.length ? values[i] : 0;
    });

    chartData.push(point);
  }

  return chartData;
}

export function PlotPanel({ timeSeries }: PlotPanelProps) {
  const [selectedPlot, setSelectedPlot] = useState<PlotType>('altitude');

  const selectedConfig = PLOT_CONFIGS.find((c) => c.id === selectedPlot);

  if (!timeSeries || !selectedConfig) {
    return (
      <div style={styles.container}>
        <div style={styles.placeholder}>
          <p style={styles.placeholderText}>
            No data available. Run the simulation to see plots.
          </p>
        </div>
      </div>
    );
  }

  const chartData = formatChartData(timeSeries, selectedConfig);

  return (
    <div style={styles.container}>
      {/* Plot selector */}
      <div style={styles.header}>
        <h3 style={styles.title} className="section-title">State Function Time Series</h3>
        <select
          style={styles.select}
          className="touch-input"
          value={selectedPlot}
          onChange={(e) => setSelectedPlot(e.target.value as PlotType)}
        >
          {PLOT_CONFIGS.map((config) => (
            <option key={config.id} value={config.id}>
              {config.title}
            </option>
          ))}
        </select>
      </div>

      {/* Chart */}
      <div style={styles.chartContainer}>
        <ResponsiveContainer width="100%" height="100%">
          <LineChart
            data={chartData}
            margin={{ top: 10, right: 30, left: 20, bottom: 10 }}
          >
            <CartesianGrid strokeDasharray="3 3" stroke="var(--bg-tertiary)" />
            <XAxis
              dataKey="time"
              stroke="var(--text-primary)"
              label={{ value: 'Time (s)', position: 'insideBottom', offset: -5 }}
            />
            <YAxis
              stroke="var(--text-primary)"
              label={{
                value: selectedConfig.yLabel,
                angle: -90,
                position: 'insideLeft',
              }}
            />
            <Tooltip
              contentStyle={{
                backgroundColor: 'var(--bg-secondary)',
                border: '1px solid var(--bg-tertiary)',
                borderRadius: '4px',
              }}
              labelStyle={{ color: 'var(--text-primary)' }}
            />
            <Legend />
            {selectedConfig.lines.map((line) => (
              <Line
                key={line.dataKey}
                type="monotone"
                dataKey={line.dataKey}
                name={line.name}
                stroke={line.color}
                strokeWidth={2}
                dot={false}
                isAnimationActive={false}
              />
            ))}
          </LineChart>
        </ResponsiveContainer>
      </div>

      {/* Info */}
      <div style={styles.footer}>
        <span style={styles.infoText}>
          Data points: {timeSeries.time.length}
        </span>
        {timeSeries.time.length > 0 && (
          <span style={styles.infoText}>
            Duration: {timeSeries.time[timeSeries.time.length - 1].toFixed(2)} s
          </span>
        )}
      </div>
    </div>
  );
}

const styles = {
  container: {
    display: 'flex',
    flexDirection: 'column' as const,
    width: '100%',
    height: '100%',
    backgroundColor: 'var(--bg-secondary)',
    borderTop: '2px solid var(--bg-tertiary)',
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '12px 20px',
    backgroundColor: 'var(--bg-tertiary)',
    borderBottom: '1px solid var(--border-color)',
  },
  title: {
    margin: 0,
    fontSize: '16px',
    fontWeight: 'bold' as const,
    color: 'var(--text-primary)',
  },
  select: {
    padding: '6px 12px',
    fontSize: '14px',
    backgroundColor: 'var(--bg-secondary)',
    color: 'var(--text-primary)',
    border: '1px solid var(--border-color)',
    borderRadius: '4px',
    cursor: 'pointer',
    outline: 'none',
  },
  chartContainer: {
    flex: 1,
    padding: '10px',
    minHeight: 0,
  },
  footer: {
    display: 'flex',
    justifyContent: 'space-between',
    padding: '8px 20px',
    backgroundColor: 'var(--bg-tertiary)',
    borderTop: '1px solid var(--border-color)',
  },
  infoText: {
    fontSize: '12px',
    color: 'var(--text-secondary)',
  },
  placeholder: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    height: '100%',
  },
  placeholderText: {
    color: 'var(--text-secondary)',
    fontSize: '14px',
  },
};
