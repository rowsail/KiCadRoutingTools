"""Clearance tiers, out-of-scope rips and the nets a run breaks off-scope.

Three defects found routing a 4-layer HV supply (a 2 mm HV class beside 0.2 mm
signals, 0402 parts on the bottom), pinned here without routing a board:

  1. One HV net in a call set the routing floor for EVERY net to 2 mm, so the
     first fine-pitch net could not leave its pads ("8/8 neighbors blocked")
     and 0 of 106 nets routed in 15 minutes. batch_route now splits a call
     that mixes classes into one pass per class, highest first
     (_clearance_tier_plan), and the tiers' tallies fold into the one
     JSON_SUMMARY_MIN / --json-out (fold_tier_summaries, and the log reader).
  2. The in-run plane finalize ripped a net the caller had '!'-negated in
     --nets to land a tap via, and the net shipped open; under
     --keep-input-copper it may rip nothing outside the run's scope
     (_finalize_never_rip).
  3. That net was named by the improvement gate and by nothing else: every
     failure key is scoped to routed nets, so the merged tally read clean.
     fold_gate_losses carries such a net into coverage_gate_nets (weight 1 in
     the deficit) and summary_min names it.
"""
import json
import os
import sys
import tempfile
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import route_summary as rs  # noqa: E402


class TestGateLosses(unittest.TestCase):
    def test_unclassified_loss_weighs_and_is_named(self):
        merged = {'successful': 5, 'failed': 0, 'failed_single': [],
                  'routed_single': ['A'], 'multipoint_pads_total': 10,
                  'multipoint_pads_connected': 10}
        out = rs.fold_gate_losses(merged, ['/EFUSE/SHDN'])
        self.assertEqual(out['coverage_gate_nets'], ['/EFUSE/SHDN'])
        self.assertEqual(out['multipoint_pads_total'], 11)
        mn = rs.summary_min(out)
        self.assertEqual(mn['multipoint_deficit'], 1)
        self.assertEqual(mn['broken_outside_scope'], ['/EFUSE/SHDN'])
        self.assertEqual(mn['broken_by_run'], ['/EFUSE/SHDN'])
        self.assertNotIn('coverage_gate_nets', merged, 'input must not be mutated')

    def test_classified_loss_not_double_counted(self):
        merged = {'failed_single': ['B'], 'multipoint_pads_total': 4,
                  'failed_multipoint': [{'net_name': 'C'}]}
        out = rs.fold_gate_losses(merged, ['B', 'C'])
        self.assertNotIn('coverage_gate_nets', out)
        self.assertEqual(out['multipoint_pads_total'], 4)
        self.assertEqual(out['broken_by_run'], ['B', 'C'])

    def test_no_losses_is_identity(self):
        m = {'failed': 0}
        self.assertIs(rs.fold_gate_losses(m, []), m)
        self.assertIsNone(rs.fold_gate_losses(None, ['X']))


class TestTierFold(unittest.TestCase):
    def test_tiers_concatenate_and_add(self):
        t1 = {'successful': 10, 'failed': 1, 'failed_single': ['/HT/HT'],
              'total_vias': 3, 'total_iterations': 100, 'total_time': 1.0,
              'multipoint_pads_total': 6, 'multipoint_pads_connected': 5,
              'min_clearance_used': 2.0, 'tier_clearance': 2.0,
              'tier_nets': ['/HT/HT', '/HT/HTRAW']}
        last = {'successful': 90, 'failed': 2, 'failed_single': ['X', 'Y'],
                'total_vias': 200, 'total_iterations': 1000, 'total_time': 9.0,
                'multipoint_pads_total': 300, 'multipoint_pads_connected': 299,
                'min_clearance_used': 0.2}
        out = rs.fold_tier_summaries(last, [t1])
        self.assertEqual(out['failed_single'], ['/HT/HT', 'X', 'Y'])
        self.assertEqual((out['successful'], out['failed']), (100, 3))
        self.assertEqual(out['total_vias'], 203)
        self.assertEqual(out['multipoint_pads_total'], 306)
        self.assertEqual(out['min_clearance_used'], 0.2)
        mn = rs.summary_min(out)
        self.assertEqual(mn['multipoint_deficit'], 2)
        self.assertEqual(mn['clearance_tiers'],
                         [{'clearance': 2.0, 'nets': 2, 'failed': 1}])

    def test_log_reader_folds_tier_lines_only(self):
        t1 = {'successful': 1, 'failed': 1, 'failed_single': ['HV'],
              'tier_clearance': 2.0, 'tier_nets': ['HV']}
        inner = {'successful': 99, 'failed': 0, 'failed_single': []}
        last = {'successful': 4, 'failed': 0, 'failed_single': []}
        log = '\n'.join([
            'JSON_TIER_PASS_SUMMARY: ' + json.dumps(inner),
            'JSON_TIER_SUMMARY_MIN (tier 1/2: 2 mm): {"failed": 1}',
            'JSON_TIER_SUMMARY: ' + json.dumps(t1),
            'JSON_SUMMARY: ' + json.dumps(last),
            'JSON_SUMMARY_MIN: {"failed": 1}'])
        # Neither pass-level tag is read as a run summary or a MIN line.
        self.assertEqual(len(rs.SUMMARY_RE.findall(log)), 1)
        self.assertEqual(len(rs.SUMMARY_MIN_RE.findall(log)), 1)
        m = rs.merge_route_summaries(log)
        self.assertEqual(m['failed_single'], ['HV'])
        self.assertEqual((m['successful'], m['failed']), (5, 1))


def _route():
    import route
    return route


class TestTierPlan(unittest.TestCase):
    def setUp(self):
        self.td = tempfile.TemporaryDirectory()
        self.inp = os.path.join(self.td.name, 'b.kicad_pcb')
        open(self.inp, 'w').write('(kicad_pcb)')
        self.route = _route()
        import list_nets
        self._orig = list_nets.net_clearance_map
        list_nets.net_clearance_map = lambda p, names, rules=None: {
            'HV1': 2.0, 'HV2': 2.0, 'SW': 1.0, 'LOW': 0.1}

    def tearDown(self):
        import list_nets
        list_nets.net_clearance_map = self._orig
        self.td.cleanup()

    def _args(self, **kw):
        a = dict(input_file=self.inp, output_file=self.inp + '.out',
                 net_names=['A', 'HV1', 'SW', 'HV2', 'B', 'LOW'],
                 final_reconcile=True, pcb_data=None, return_results=False,
                 skip_routing=False, net_clearances=None, clearance=0.2,
                 clearance_ceiling=None)
        a.update(kw)
        return a

    def test_mixed_classes_tier_highest_first(self):
        plan = self.route._clearance_tier_plan(self._args())
        self.assertEqual(plan, [(2.0, ['HV1', 'HV2']), (1.0, ['SW']),
                                (0.2, ['A', 'B', 'LOW'])])

    def test_ceiling_and_uniform_calls_do_not_tier(self):
        self.assertIsNone(self.route._clearance_tier_plan(
            self._args(clearance_ceiling=0.2)))
        self.assertIsNone(self.route._clearance_tier_plan(
            self._args(net_names=['A', 'B', 'LOW'])))
        self.assertEqual(self.route._clearance_tier_plan(
            self._args(clearance_ceiling=1.0))[0], (1.0, ['HV1', 'SW', 'HV2']))

    def test_only_outermost_file_runs_tier(self):
        for kw in ({'final_reconcile': False}, {'pcb_data': object()},
                   {'return_results': True}, {'skip_routing': True},
                   {'net_clearances': {}}):
            self.assertIsNone(self.route._clearance_tier_plan(self._args(**kw)), kw)

    def test_knob_and_nesting_switch_it_off(self):
        import env_knobs
        old = env_knobs.CLEARANCE_TIERS
        try:
            env_knobs.CLEARANCE_TIERS = False
            self.assertIsNone(self.route._clearance_tier_plan(self._args()))
        finally:
            env_knobs.CLEARANCE_TIERS = old
        self.route._TIER_PASS.append('tier 1/2')
        try:
            self.assertIsNone(self.route._clearance_tier_plan(self._args()))
        finally:
            self.route._TIER_PASS.pop()


class TestFinalizeNeverRip(unittest.TestCase):
    def _board(self):
        N = lambda name: types.SimpleNamespace(name=name)
        S = lambda nid: types.SimpleNamespace(net_id=nid)
        return types.SimpleNamespace(
            nets={1: N('GND'), 2: N('/EFUSE/SHDN'), 3: N('SIG'), 4: N('OTHER'),
                  5: N('!RESET')},
            segments=[S(2), S(3), S(4), S(5)], vias=[S(1)])

    def test_negated_nets_are_hands_off(self):
        r = _route()
        got = r._finalize_never_rip(self._board(), ['SIG'],
                                    ['*', '!/EFUSE/SHDN', '!RESET'], False, {3})
        # '!RESET' names a real active-low net, so it is a literal include.
        self.assertEqual(got, ['/EFUSE/SHDN'])

    def test_keep_input_copper_freezes_everything_out_of_scope(self):
        r = _route()
        got = r._finalize_never_rip(self._board(), ['SIG'], None, True, {3})
        self.assertEqual(got, ['!RESET', '/EFUSE/SHDN', 'GND', 'OTHER'])


if __name__ == '__main__':
    unittest.main()
