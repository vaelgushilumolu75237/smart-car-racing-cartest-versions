"""Use actual topology evidence, including pure ISLAND's ambiguous flag."""
import unittest
from unittest.mock import patch
import test_ring_exit_pending_route as pending
import config
import track_elements as te
from fast_topology import FastTopologyGuard


class RingIslandTopologyContractTests(unittest.TestCase):
    def setup_ring(self, side=te.SIDE_LEFT):
        h=pending.PendingExitRouteTests().tracker(side)
        h.tracker._state=te.STATE_INSIDE
        return h

    def island_mask(self,h):
        m=h.mask()
        cx=47 if h.tracker._side==te.SIDE_LEFT else 33
        for y in range(20,29):
            for x in range(cx-4,cx+5):
                radius=((x-cx)**2+(y-24)**2)**.5
                if 2.4<=radius<=3.5:
                    m['road_mask'][y*80+x]=0
        m['edge_mask']=bytearray(0 if v else 1 for v in m['road_mask'])
        m['row_counts']=bytearray(sum(m['edge_mask'][y*80:(y+1)*80]) for y in range(60))
        evidence=dict(FastTopologyGuard(config).check(m))
        self.assertTrue(evidence['island_evidence'])
        self.assertTrue(evidence['ambiguous'])
        self.assertFalse(evidence['branch_evidence'])
        self.assertEqual(evidence['reason'],'ISLAND')
        return m,evidence

    def test_actual_pure_island_evidence_keeps_a_qualified_inner_route(self):
        for side in (te.SIDE_LEFT,te.SIDE_RIGHT):
            h=self.setup_ring(side);mask,evidence=self.island_mask(h)
            r=h.frame(road_accepted=False,raw_side=side,
                      topology=evidence,mask_info=mask)
            self.assertEqual(r['state_id'],te.STATE_INSIDE)
            self.assertTrue(r['target_valid'])
            self.assertTrue(r['control_eligible'])

    def test_island_exception_does_not_open_entry_states(self):
        for state in (te.STATE_OBSERVE,te.STATE_ENTRY_CONFIRM,te.STATE_ENTER):
            h=self.setup_ring();_,evidence=self.island_mask(h)
            h.tracker._state=state
            self.assertFalse(h.tracker._ring_topology_precheck(evidence))

    def test_island_does_not_skip_independent_branch_kernel_or_trust_old_output(self):
        h=self.setup_ring();mask,evidence=self.island_mask(h)
        self.assertTrue(evidence['independent_branch_checked'])
        evidence.pop('independent_branch_checked')
        self.assertFalse(h.tracker._ring_topology_precheck(evidence))
        with patch.object(FastTopologyGuard,'_open_band_host',return_value=True) as kernel:
            evidence=dict(FastTopologyGuard(config).check(mask))
        kernel.assert_called()
        self.assertTrue(evidence['island_evidence'])
        self.assertTrue(evidence['branch_evidence'])
        self.assertEqual(h.tracker._ring_topology_rejection(evidence),'BRANCH')

    def test_independent_branch_and_multiple_exit_flags_still_fail_closed(self):
        for name in ('branch_evidence','branch','independent_branch_ambiguous',
                     'multi_branch_ambiguous','multiple_exits'):
            h=self.setup_ring();mask,evidence=self.island_mask(h)
            evidence[name]=True
            r=h.frame(road_accepted=False,raw_side=te.SIDE_LEFT,
                      topology=evidence,mask_info=mask)
            self.assertFalse(r['target_valid'],name)
            expected='MULTIPLE_EXITS' if name=='multiple_exits' else 'BRANCH'
            self.assertEqual(r['reason'],'RING_TOPOLOGY_'+expected)

    def test_incomplete_unknown_or_unfrozen_context_is_not_pure_ring_evidence(self):
        for mode in ('incomplete','unknown_reason','unfrozen','uncalibrated'):
            h=self.setup_ring();_,evidence=self.island_mask(h)
            if mode=='incomplete':evidence['complete']=False
            elif mode=='unknown_reason':evidence['reason']='UNKNOWN_REGION'
            elif mode=='unfrozen':h.tracker._ring_cache_frozen=False
            else:h.tracker._cal_check_ready=False
            self.assertFalse(h.tracker._ring_topology_precheck(evidence),mode)

    def test_pure_island_does_not_authorize_a_broken_centerline_corridor(self):
        h=self.setup_ring();mask,evidence=self.island_mask(h)
        mask['road_mask'][40*80+35]=0
        r=h.frame(road_accepted=False,raw_side=te.SIDE_LEFT,
                  topology=evidence,mask_info=mask)
        self.assertFalse(r['target_valid'])
        self.assertFalse(r['control_eligible'])
        self.assertEqual(r['reason'],'RING_TOPOLOGY_ROUTE_UNSUPPORTED')

    def test_unknown_ambiguity_remains_rejected_without_an_island(self):
        h=self.setup_ring()
        evidence=dict(complete=True,ambiguous=True,reason='OK')
        r=h.frame(road_accepted=False,raw_side=te.SIDE_LEFT,topology=evidence)
        self.assertFalse(r['target_valid'])
        self.assertEqual(r['reason'],'RING_TOPOLOGY_AMBIGUOUS')

    def test_mixed_bundle_without_the_new_contract_is_rejected(self):
        import test_fast_main_ring_slope
        import main
        with patch.object(te,'RING_ISLAND_TOPOLOGY_API_VERSION',0):
            with self.assertRaisesRegex(RuntimeError,'island-topology API'):
                main._load_track_elements_runtime()


if __name__=='__main__':unittest.main()
