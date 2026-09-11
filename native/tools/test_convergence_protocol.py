import copy
import unittest
import acceptance as acc
from test_acceptance import strict_obj, case, fixture_report

class ConvergenceProtocolTests(unittest.TestCase):
    def rejected(self, mutate, expected):
        obj = strict_obj(); mutate(obj)
        kind, _, reason = acc.strict_report_check(obj, case())
        self.assertEqual((kind, reason), ('invalid', expected))

    def test_malformed_types_return_invalid(self):
        self.rejected(lambda o: o['convergence']['metrics'][0].__setitem__('name', []), 'CONVERGENCE_METRICS')
        self.rejected(lambda o: o['convergence']['metrics'][0].__setitem__('coarse', 10**400), 'CONVERGENCE_NONFINITE')
        self.rejected(lambda o: o.__setitem__('limits', {'maxLateralRateGps': 10**400}), 'CONVERGENCE_LIMITS')

    def test_missing_assessment(self):
        self.rejected(lambda o: o.pop('convergence'), 'CONVERGENCE_NOT_VERIFIED')

    def test_unperformed_or_failed(self):
        for key in ('performed', 'passed'):
            with self.subTest(key=key):
                self.rejected(lambda o: o['convergence'].__setitem__(key, False), 'CONVERGENCE_NOT_VERIFIED')

    def test_same_timestep_is_not_half_step(self):
        self.rejected(lambda o: o['convergence'].__setitem__('fineStep', .01), 'CONVERGENCE_STEP')

    def test_different_request_step_rejected(self):
        self.rejected(lambda o: o['convergence'].__setitem__('coarseStep', .02), 'CONVERGENCE_STEP')

    def test_matching_obsolete_rates_are_still_rejected(self):
        o = strict_obj(); c = case(); c['step'] = .01
        o['convergence'].update(coarseStep=.01, fineStep=.005)
        self.assertEqual(acc.strict_report_check(o, c)[2], 'CONVERGENCE_STEP')

    def test_envelope_evidence_is_required_and_bound(self):
        self.rejected(lambda o: o.pop('forceEnvelope'), 'CONVERGENCE_ENVELOPE_BINDING')
        self.rejected(lambda o: o['forceEnvelope']['seats'][0]['directionalG'].pop(), 'CONVERGENCE_ENVELOPE_BINDING')
        self.rejected(lambda o: o['forceEnvelope']['seats'][1].__setitem__('passed', False), 'CONVERGENCE_ENVELOPE_BINDING')
        self.rejected(lambda o: o['forceEnvelope']['seats'][2]['enhancedLongitudinalOnsetGps'].__setitem__('utilization', .1), 'CONVERGENCE_COARSE_BINDING')

    def test_envelope_rows_cannot_be_omitted(self):
        self.rejected(lambda o: o['convergence'].__setitem__('metrics', [r for r in o['convergence']['metrics'] if '.forceEnvelope.' not in r['name']]), 'CONVERGENCE_METRICS')

    def test_omitted_rear_statistic(self):
        self.rejected(lambda o: o['convergence']['metrics'].pop(), 'CONVERGENCE_METRICS')

    def test_duplicate_does_not_replace_missing_seat(self):
        self.rejected(lambda o: o['convergence']['metrics'].__setitem__(-1, copy.deepcopy(o['convergence']['metrics'][0])), 'CONVERGENCE_METRICS')

    def test_forged_difference(self):
        self.rejected(lambda o: o['convergence']['metrics'][1].__setitem__('fine', .01), 'CONVERGENCE_ARITHMETIC')

    def test_inflated_tolerance(self):
        self.rejected(lambda o: o['convergence']['metrics'][1].__setitem__('tolerance', 1.), 'CONVERGENCE_ARITHMETIC')

    def test_exact_tolerance_boundary_fails(self):
        def mutate(o): o['convergence']['metrics'][1].update(fine=.02, absoluteDifference=.02)
        self.rejected(mutate, 'CONVERGENCE_TOLERANCE')

    def test_nonfinite_statistic(self):
        self.rejected(lambda o: o['convergence']['metrics'][1].__setitem__('fine', float('nan')), 'CONVERGENCE_NONFINITE')

    def test_forged_summary(self):
        self.rejected(lambda o: o['convergence'].__setitem__('maxForceRelativeError', .01), 'CONVERGENCE_SUMMARY')

    def test_malformed_optional_limit_rejected(self):
        for value in (float('nan'), float('inf'), -1., 0., True, 'unassessed'):
            with self.subTest(value=value):
                self.rejected(lambda o: o.__setitem__('limits', {'maxLateralRateGps': value}), 'CONVERGENCE_LIMITS')

    def test_coarse_evidence_bound_to_report(self):
        self.rejected(lambda o: o['metrics'].__setitem__('maxSpeed', 80.), 'CONVERGENCE_COARSE_BINDING')
        self.rejected(lambda o: o['seatStatistics'][2]['axes'][1].__setitem__('maxG', 1.), 'CONVERGENCE_COARSE_BINDING')

    def test_roundtrip_doubles_preserve_subthreshold_result(self):
        import json
        o = strict_obj(); r = o['convergence']['metrics'][1]
        r.update(coarse=1.0199999999999, fine=1., absoluteDifference=abs(1.0199999999999-1.), tolerance=.02)
        o['convergence']['maxForceRelativeError'] = r['absoluteDifference']
        fixture_report(o)
        self.assertEqual(acc.strict_report_check(json.loads(json.dumps(o)), case())[0], 'accepted-candidate')

    def test_configured_rate_requires_all_seats(self):
        self.rejected(lambda o: o.__setitem__('limits', {'maxLateralRateGps': 10}), 'CONVERGENCE_METRICS')
        o = strict_obj(); o['limits'] = {'maxLateralRateGps': 10}
        for seat in ('front', 'middle', 'rear'):
            o['convergence']['metrics'].append({'name': seat + '.lateral.maxRateGps', 'coarse': 1., 'fine': 1., 'absoluteDifference': 0., 'tolerance': .02})
        self.assertEqual(acc.strict_report_check(fixture_report(o), case())[0], 'accepted-candidate')

if __name__ == '__main__': unittest.main()
