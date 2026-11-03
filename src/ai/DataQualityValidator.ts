/**
 * @file DataQualityValidator.ts
 * Automated dataset quality verification and integrity inspection (Phase H).
 * Detects numerical anomalies, NaNs, infinities, out-of-bound joint limits,
 * temporal jitter, timestamp drift, and discontinuous spatial teleportation.
 */

import { DigitalTwinObservation } from './ObservationBuilder.js';
import { EpisodeStepFrame, RecordedEpisodeDataset } from './EpisodeRecorder.js';
import { DEFAULT_ROBOT_CONFIGURATION } from '../robot/RobotConfiguration.js';
import { RAD2DEG } from '../robot/RobotJoints.js';

export interface EpisodeAnomalyReport {
  episode_id: string;
  has_nan_or_inf: boolean;
  nan_fields: string[];
  has_joint_limit_violations: boolean;
  joint_violation_count: number;
  has_temporal_anomalies: boolean;
  temporal_issues: string[];
  has_discontinuous_jumps: boolean;
  discontinuous_jump_count: number;
  is_dataset_compliant: boolean;
  compliance_score: number; // 0.0 to 1.0
}

export interface DatasetQualityReport {
  generated_at: string;
  total_episodes_inspected: number;
  compliant_episodes_count: number;
  non_compliant_episodes_count: number;
  compliance_rate_percent: number;
  average_steps_per_episode: number;
  average_duration_seconds: number;
  average_final_position_error_mm: number;
  total_nan_occurrences: number;
  total_joint_limit_violations: number;
  total_temporal_discontinuities: number;
  episodes_summary: EpisodeAnomalyReport[];
}

export class DataQualityValidator {
  public maxAllowedCartesianJumpMeters: number = 0.12;
  public expectedDtSeconds: number = 0.05;
  public dtJitterToleranceSeconds: number = 0.025;

  /**
   * Inspects a single recorded episode dataset for data integrity.
   */
  public inspectEpisode(episode: RecordedEpisodeDataset): EpisodeAnomalyReport {
    const nanFields: string[] = [];
    const temporalIssues: string[] = [];
    let jointViolationCount = 0;
    let jumpCount = 0;

    const config = DEFAULT_ROBOT_CONFIGURATION;

    for (let i = 0; i < episode.frames.length; i++) {
      const frame = episode.frames[i];
      const obs = frame.observation;

      // 1. NaN and Infinity checks
      this._checkNumericalArray(obs.joint_positions_rad, `frame[${i}].joint_positions_rad`, nanFields);
      this._checkNumericalArray(obs.joint_velocities_rad_s, `frame[${i}].joint_velocities_rad_s`, nanFields);
      this._checkNumericalArray(obs.end_effector_position, `frame[${i}].end_effector_position`, nanFields);
      this._checkNumericalArray(obs.end_effector_orientation, `frame[${i}].end_effector_orientation`, nanFields);
      this._checkNumericalArray(obs.object_position, `frame[${i}].object_position`, nanFields);
      this._checkNumericalArray(obs.object_orientation, `frame[${i}].object_orientation`, nanFields);

      // 2. Joint limit verification
      obs.joint_positions_rad.forEach((rad, jIdx) => {
        const deg = rad * RAD2DEG;
        const cfg = config.joints[jIdx];
        if (cfg) {
          if (deg < cfg.minAngleDeg - 1.5 || deg > cfg.maxAngleDeg + 1.5) {
            jointViolationCount++;
          }
        }
      });

      // 3. Temporal consistency and monotonic timestamps
      if (i > 0) {
        const prevFrame = episode.frames[i - 1];
        const dt = frame.timestamp - prevFrame.timestamp;
        if (dt <= 0) {
          temporalIssues.push(`Non-monotonic timestamp at step ${i}: dt=${dt.toFixed(4)}s.`);
        } else if (Math.abs(dt - this.expectedDtSeconds) > this.dtJitterToleranceSeconds) {
          temporalIssues.push(`Timestamp jitter at step ${i}: dt=${dt.toFixed(4)}s (expected ~${this.expectedDtSeconds}s).`);
        }

        // 4. Discontinuity detection (teleportation jumps)
        const prevTcp = prevFrame.end_effector_pose.position;
        const curTcp = frame.end_effector_pose.position;
        const tcpJump = Math.hypot(curTcp[0] - prevTcp[0], curTcp[1] - prevTcp[1], curTcp[2] - prevTcp[2]);
        if (tcpJump > this.maxAllowedCartesianJumpMeters) {
          jumpCount++;
        }
      }
    }

    const hasNan = nanFields.length > 0;
    const hasJointViolations = jointViolationCount > 0;
    const hasTemporal = temporalIssues.length > 0;
    const hasJumps = jumpCount > 0;

    const isCompliant = !hasNan && !hasJointViolations && !hasJumps && episode.frames.length >= 10;
    let score = 1.0;
    if (hasNan) score -= 0.5;
    if (hasJointViolations) score -= 0.2;
    if (hasTemporal) score -= 0.15;
    if (hasJumps) score -= 0.15;
    score = Math.max(0, score);

    return {
      episode_id: episode.episode_id,
      has_nan_or_inf: hasNan,
      nan_fields: nanFields.slice(0, 10), // Limit summary
      has_joint_limit_violations: hasJointViolations,
      joint_violation_count: jointViolationCount,
      has_temporal_anomalies: hasTemporal,
      temporal_issues: temporalIssues.slice(0, 5),
      has_discontinuous_jumps: hasJumps,
      discontinuous_jump_count: jumpCount,
      is_dataset_compliant: isCompliant,
      compliance_score: Math.round(score * 100) / 100
    };
  }

  /**
   * Inspects a batch of recorded episodes and outputs a consolidated quality report.
   */
  public generateDatasetQualityReport(episodes: RecordedEpisodeDataset[]): DatasetQualityReport {
    if (!episodes || episodes.length === 0) {
      return {
        generated_at: new Date().toISOString(),
        total_episodes_inspected: 0,
        compliant_episodes_count: 0,
        non_compliant_episodes_count: 0,
        compliance_rate_percent: 0,
        average_steps_per_episode: 0,
        average_duration_seconds: 0,
        average_final_position_error_mm: 0,
        total_nan_occurrences: 0,
        total_joint_limit_violations: 0,
        total_temporal_discontinuities: 0,
        episodes_summary: []
      };
    }

    const summaries = episodes.map((ep) => this.inspectEpisode(ep));
    const compliantCount = summaries.filter((s) => s.is_dataset_compliant).length;

    let totalSteps = 0;
    let totalDuration = 0;
    let totalPosErrM = 0;
    let totalNans = 0;
    let totalJointViolations = 0;
    let totalDiscontinuities = 0;

    episodes.forEach((ep, i) => {
      totalSteps += ep.total_steps;
      const first = ep.frames[0];
      const last = ep.frames[ep.frames.length - 1];
      if (first && last) {
        totalDuration += last.timestamp - first.timestamp;
      }
      if (ep.validation_report) {
        totalPosErrM += ep.validation_report.metrics.final_position_error_m;
      }
      const sum = summaries[i];
      if (sum.has_nan_or_inf) totalNans++;
      totalJointViolations += sum.joint_violation_count;
      totalDiscontinuities += sum.discontinuous_jump_count;
    });

    const n = episodes.length;
    return {
      generated_at: new Date().toISOString(),
      total_episodes_inspected: n,
      compliant_episodes_count: compliantCount,
      non_compliant_episodes_count: n - compliantCount,
      compliance_rate_percent: Math.round((compliantCount / n) * 10000) / 100,
      average_steps_per_episode: Math.round(totalSteps / n),
      average_duration_seconds: Math.round((totalDuration / n) * 100) / 100,
      average_final_position_error_mm: Math.round((totalPosErrM / n) * 1000 * 10) / 10,
      total_nan_occurrences: totalNans,
      total_joint_limit_violations: totalJointViolations,
      total_temporal_discontinuities: totalDiscontinuities,
      episodes_summary: summaries
    };
  }

  private _checkNumericalArray(arr: number[] | undefined, name: string, list: string[]): void {
    if (!arr) return;
    for (let i = 0; i < arr.length; i++) {
      if (!Number.isFinite(arr[i])) {
        list.push(`${name}[${i}] = ${arr[i]}`);
      }
    }
  }
}
