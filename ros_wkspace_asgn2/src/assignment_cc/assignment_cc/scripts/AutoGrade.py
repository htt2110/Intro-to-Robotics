#!/usr/bin/env python3
# coding: UTF-8
import sys
l1ll111_opy_ = sys.version_info [0] == 2
l1111l11_opy_ = 2048
l1l1l1_opy_ = 7
def l1l1ll11_opy_ (l111111l_opy_):
    global l11111l_opy_
    l1llll11l_opy_ = ord (l111111l_opy_ [-1])
    l1llll11_opy_ = l111111l_opy_ [:-1]
    l11ll1l1_opy_ = l1llll11l_opy_ % len (l1llll11_opy_)
    l1111lll_opy_ = l1llll11_opy_ [:l11ll1l1_opy_] + l1llll11_opy_ [l11ll1l1_opy_:]
    if l1ll111_opy_:
        l1lll1_opy_ = unicode () .join ([unichr (ord (char) - l1111l11_opy_ - (l1lll1ll_opy_ + l1llll11l_opy_) % l1l1l1_opy_) for l1lll1ll_opy_, char in enumerate (l1111lll_opy_)])
    else:
        l1lll1_opy_ = str () .join ([chr (ord (char) - l1111l11_opy_ - (l1lll1ll_opy_ + l1llll11l_opy_) % l1l1l1_opy_) for l1lll1ll_opy_, char in enumerate (l1111lll_opy_)])
    return eval (l1lll1_opy_)
(lambda __operator, __print, __g: [(sys.path.append(l1l1ll11_opy_('ࠢ\u082fࠤ\u088f')), [[[(lambda __mod: [[[[[(lambda __after: (rospy.init_node(l1l1ll11_opy_('ࠨࡥࡤ\u0876\u0879\u086b\u0873ࡪࡣ\u0871ࡣ\u086c\u0878ࡡࡥࡧ\u0875ࠫ\u089e'), anonymous=True), (__print(l1l1ll11_opy_('ࠤࡌ\u0872\u086e\u087aࡩࡢ\u086e\u086c\u087e\u086e\u0874ࡧࠣ\u089f')), [[(rospy.sleep(1.0), (__print(l1l1ll11_opy_('ࠥࡖࡪ\u0879ࡥ\u0875\u0876\u086c\u0872\u086cࠦ\u0872\u0870ࡤ\u0872\u0878ࠧࢠ')), (l1l1l1l11_opy_.reset_robot(), (rospy.sleep(1.0), [[(lambda __after: [(__print(l1l1ll11_opy_('ࠦࡎࡑ࠺ࠡࡒࡄࡗࡘࡋࡄࠣࢡ')), __after())[1] for __g['l1l1ll11l_opy_'] in [(__operator.iadd(__g['l1l1ll11l_opy_'], 2.5))]][0] if (l1l1llll1_opy_ == 4) else [(lambda __after: (__print(l1l1ll11_opy_('ࠧࡏࡋ࠻ࠢࡓࡅࡘ࡙ࡅࡅࠤࢢ')), [__after() for __g['l1l1ll11l_opy_'] in [(__operator.iadd(__g['l1l1ll11l_opy_'], 2.5))]][0])[1] if (l1l1l1l1l_opy_ == 4) else (__print(l1l1ll11_opy_('ࠨࡉࡌࠢࡉࡅࡎࡒࡅࡅࠤࢣ')), __after())[1])(lambda: __after()) for __g['l1l1l1l1l_opy_'] in [(l1ll11111_opy_(robot))]][0])(lambda: (rospy.sleep(1.0), (l1l1l1l11_opy_.reset_robot(), (rospy.sleep(1.0), [(lambda __after: (__print(l1l1ll11_opy_('ࠣࡅࡤ\u0876\u0879\u086b\u0873ࡪࡣ\u0871ࠤࡨ\u0875\u086e\u0875\u0874\u0872\u0870\u083fࠦࡐࡂࡕࡖࡉࡉࠨࢥ')), __after())[1] if (l1l1l1lll_opy_ == 5) else (__print(l1l1ll11_opy_('ࠤࡆࡥ\u0877\u087aࡥ\u0874\u086bࡤ\u0872ࠥࡩ\u086f\u086f\u0876\u0875\u0873\u0871ࡀࠠࡇࡃࡌࡐࡊࡊࠢࢦ')), (lambda __after: (__print(l1l1ll11_opy_('ࠥࡆࡔ࡚ࡈࠡ\u0874\u0872\u0878ࡦ\u087aࡩ\u0870\u0870ࠣࡥ\u0873ࡪࠠ\u0875\u0874ࡤ\u0872\u0878\u0872ࡡ\u0875\u086b\u0872\u0872ࠥ\u086cࡡࡪ\u086eࡨࡨࠧࢧ')), __after())[1] if (l1l1l1lll_opy_ == 0) else (lambda __after: (__print(l1l1ll11_opy_('ࠦࡘࡏࡍࡖࡎࡗࡅࡓࡋࡏࡖࡕࠣ\u0876\u0874\u087aࡡ\u0875\u086b\u0872\u0872ࠥࡧ\u086eࡥࠢ\u0877\u0876ࡦ\u0874\u0873\u086dࡣ\u0877\u086d\u0874\u0874ࠠࡧࡣ\u086c\u0870ࡪࡪࠢࢨ')), __after())[1] if (l1l1l1lll_opy_ == 3) else (__print(l1l1ll11_opy_('ࠧࡋࡉࡕࡊࡈࡖࠥ\u0878\u086f\u0875ࡣ\u0877\u086d\u0874\u0874ࠠࡐࡔࠣ\u0878\u0877ࡧ\u086e\u0874\u086eࡤ\u0878\u086e\u0875\u086eࠡࡨࡤ\u086d\u0871\u086bࡤࠣࢩ')), __after())[1])(lambda: __after()))(lambda: __after()))[1])(lambda: (lambda __after: (__print(l1l1ll11_opy_('ࠨࡓࡦࡥ\u0872\u0872ࡩࡧ\u0872\u087aࠢ\u0872ࡦ\u086f\u086bࡣ\u0875\u086b\u0879ࡩ\u083fࠦࡐࡂࡕࡖࡉࡉࠨࢪ')), __after())[1] if (l1l1ll111_opy_ == 2.5) else (__print(l1l1ll11_opy_('ࠢࡔࡧࡦ\u0873\u0873ࡪࡡ\u0873\u087bࠣ\u0873ࡧ\u0870ࡥࡤ\u0876\u086c\u087aࡪࡀࠠࡇࡃࡌࡐࡊࡊࠢࢫ')), __after())[1])(lambda: [[(__print(l1l1ll11_opy_('ࠣࡉ\u0875ࡥࡩ\u086b࠺ࠡࠤࢬ'), l1l1ll11l_opy_), __after())[1] for __g['l1l1ll11l_opy_'] in [(__operator.iadd(__g['l1l1ll11l_opy_'], l1l1ll111_opy_))]][0] for __g['l1l1ll11l_opy_'] in [(__operator.iadd(__g['l1l1ll11l_opy_'], l1l1l1lll_opy_))]][0])) for (__g['l1l1l1lll_opy_'], __g['l1l1ll111_opy_']) in [(l1ll1l111_opy_(robot))]][0])[1])[1])[1]) for __g['l1l1llll1_opy_'] in [(l1ll11111_opy_(robot))]][0] for __g['l1l1ll11l_opy_'] in [(0)]][0])[1])[1])[1])[1] for __g['robot'] in [(l1l1l1l11_opy_.robot.name)]][0] for __g['l1l1l1l11_opy_'] in [(CartesianGrader())]][0])[1])[1] if (__name__ == l1l1ll11_opy_('ࠧࡠࡡ\u0870ࡥ\u086e\u0874\u085fࡠࠩ\u089d')) else __after())(lambda: None) for __g['l1ll1l111_opy_'], l1ll1l111_opy_.__name__ in [(lambda robot: (lambda __l: [[(lambda __after: [__after() for (__l['l1lll1l11_opy_'], __l['l1ll11l1l_opy_'], __l['l1ll11ll1_opy_'], __l['l1l1lll1l_opy_'], __l['l1ll1l11l_opy_'], __l['l1l1ll1l1_opy_'], __l['l1ll1111l_opy_'], __l['l1ll111l1_opy_']) in [(l1l1ll1ll_opy_())]][0] if (__l['robot'] == l1l1ll11_opy_('ࠧ\u0876\u0874࠸ࠫ\u0896')) else [__after() for (__l['l1lll1l11_opy_'], __l['l1ll11l1l_opy_'], __l['l1ll11ll1_opy_'], __l['l1l1lll1l_opy_'], __l['l1ll1l11l_opy_'], __l['l1l1ll1l1_opy_'], __l['l1ll1111l_opy_'], __l['l1ll111l1_opy_']) in [(l1l1lllll_opy_())]][0])(lambda: (__print(l1l1ll11_opy_('ࠣࡖࡨ\u0877\u0879\u086f\u086eࡨࠢࡦࡥ\u0877\u087aࡥ\u0874\u086bࡤ\u0872ࠥࡩ\u086f\u086f\u0876\u0875\u0873\u0871ࠨ\u0897')), [[[[[[[[[[[[(__l['l1l1l1lll_opy_'], __l['l1l1ll111_opy_']) for __l['l1l1ll111_opy_'] in [(l1l1l1l11_opy_.go_to_pose(l1l1ll11_opy_('ࠨࡓࡐ࠳ࠥ\u089c'), numpy.dot(__l['trans'], __l['rot']), 1, 1, 20, 2.5))]][0] for __l['rot'] in [(tf.transformations.quaternion_matrix(__l['l1l1ll1l1_opy_']))]][0] for __l['trans'] in [(tf.transformations.translation_matrix(__l['l1ll1l11l_opy_']))]][0] for __l['l1l1l1lll_opy_'] in [(__operator.iadd(__l['l1l1l1lll_opy_'], l1l1l1l11_opy_.go_to_pose(l1l1ll11_opy_('ࠦࡈࡉ࠳ࠣ\u089a'), numpy.dot(__l['trans'], __l['rot']), 0, 0, 10, 2)))]][0] for __l['rot'] in [(tf.transformations.quaternion_matrix(__l['l1l1ll1l1_opy_']))]][0] for __l['trans'] in [(tf.transformations.translation_matrix(__l['l1ll1l11l_opy_']))]][0] for __l['l1l1l1lll_opy_'] in [(__operator.iadd(__l['l1l1l1lll_opy_'], l1l1l1l11_opy_.go_to_pose(l1l1ll11_opy_('ࠥࡇࡈ࠸ࠢ\u0899'), numpy.dot(__l['trans'], __l['rot']), 0, 0, 10, 1.5)))]][0] for __l['rot'] in [(tf.transformations.quaternion_matrix(__l['l1l1lll1l_opy_']))]][0] for __l['trans'] in [(tf.transformations.translation_matrix(__l['l1ll11ll1_opy_']))]][0] for __l['l1l1l1lll_opy_'] in [(__operator.iadd(__l['l1l1l1lll_opy_'], l1l1l1l11_opy_.go_to_pose(l1l1ll11_opy_('ࠤࡆࡇ࠶ࠨ\u0898'), numpy.dot(__l['trans'], __l['rot']), 0, 0, 10, 1.5)))]][0] for __l['rot'] in [(tf.transformations.quaternion_matrix(__l['l1ll11l1l_opy_']))]][0] for __l['trans'] in [(tf.transformations.translation_matrix(__l['l1lll1l11_opy_']))]][0])[1]) for __l['l1l1l1lll_opy_'] in [(0)]][0] for __l['robot'] in [(robot)]][0])({}), 'l1ll1l111_opy_')]][0] for __g['l1ll11111_opy_'], l1ll11111_opy_.__name__ in [(lambda robot: (lambda __l: [[(lambda __after: [__after() for (__l['l1lll1l11_opy_'], __l['l1ll11l1l_opy_'], __l['l1ll11ll1_opy_'], __l['l1l1lll1l_opy_'], __l['l1ll1l11l_opy_'], __l['l1l1ll1l1_opy_'], __l['l1ll1111l_opy_'], __l['l1ll111l1_opy_']) in [(l1l1ll1ll_opy_())]][0] if (__l['robot'] == l1l1ll11_opy_('ࠨ\u0877\u0875࠹ࠬ\u0890')) else [__after() for (__l['l1lll1l11_opy_'], __l['l1ll11l1l_opy_'], __l['l1ll11ll1_opy_'], __l['l1l1lll1l_opy_'], __l['l1ll1l11l_opy_'], __l['l1l1ll1l1_opy_'], __l['l1ll1111l_opy_'], __l['l1ll111l1_opy_']) in [(l1l1lllll_opy_())]][0])(lambda: (__print(l1l1ll11_opy_('ࠤࡓࡩ\u0877\u086c\u086f\u0873\u086f\u086c\u0872\u086cࠦࡉࡌࠤ\u0891')), [[[[[[[[[[[[__l['l1l1llll1_opy_'] for __l['l1l1llll1_opy_'] in [(__operator.iadd(__l['l1l1llll1_opy_'], l1l1l1l11_opy_.go_to_ik_pose(l1l1ll11_opy_('ࠨࡉࡌ࠶ࠥ\u0895'), numpy.dot(__l['trans'], __l['rot']), 10)))]][0] for __l['rot'] in [(tf.transformations.quaternion_matrix(__l['l1ll111l1_opy_']))]][0] for __l['trans'] in [(tf.transformations.translation_matrix(__l['l1ll1111l_opy_']))]][0] for __l['l1l1llll1_opy_'] in [(__operator.iadd(__l['l1l1llll1_opy_'], l1l1l1l11_opy_.go_to_ik_pose(l1l1ll11_opy_('ࠧࡏࡋ࠴ࠤ\u0894'), numpy.dot(__l['trans'], __l['rot']), 10)))]][0] for __l['rot'] in [(tf.transformations.quaternion_matrix(__l['l1l1ll1l1_opy_']))]][0] for __l['trans'] in [(tf.transformations.translation_matrix(__l['l1ll1l11l_opy_']))]][0] for __l['l1l1llll1_opy_'] in [(__operator.iadd(__l['l1l1llll1_opy_'], l1l1l1l11_opy_.go_to_ik_pose(l1l1ll11_opy_('ࠦࡎࡑ࠲ࠣ\u0893'), numpy.dot(__l['trans'], __l['rot']), 10)))]][0] for __l['rot'] in [(tf.transformations.quaternion_matrix(__l['l1l1lll1l_opy_']))]][0] for __l['trans'] in [(tf.transformations.translation_matrix(__l['l1ll11ll1_opy_']))]][0] for __l['l1l1llll1_opy_'] in [(__operator.iadd(__l['l1l1llll1_opy_'], l1l1l1l11_opy_.go_to_ik_pose(l1l1ll11_opy_('ࠥࡍࡐ࠷ࠢ\u0892'), numpy.dot(__l['trans'], __l['rot']), 10)))]][0] for __l['rot'] in [(tf.transformations.quaternion_matrix(__l['l1ll11l1l_opy_']))]][0] for __l['trans'] in [(tf.transformations.translation_matrix(__l['l1lll1l11_opy_']))]][0])[1]) for __l['l1l1llll1_opy_'] in [(0)]][0] for __l['robot'] in [(robot)]][0])({}), 'l1ll11111_opy_')]][0] for __g['l1l1ll1ll_opy_'], l1l1ll1ll_opy_.__name__ in [(lambda : (lambda __l: [[[[[[[[(__l['l1lll1l11_opy_'], __l['l1ll11l1l_opy_'], __l['l1ll11ll1_opy_'], __l['l1l1lll1l_opy_'], __l['l1ll1l11l_opy_'], __l['l1l1ll1l1_opy_'], __l['l1ll1111l_opy_'], __l['l1ll111l1_opy_']) for __l['l1ll111l1_opy_'] in [((0.7, 0.6, (-0.3), (-0.1)))]][0] for __l['l1ll1111l_opy_'] in [(((-0.3), (-0.2), 0.3))]][0] for __l['l1l1ll1l1_opy_'] in [((0.7, 0.6, (-0.3), 0.0))]][0] for __l['l1ll1l11l_opy_'] in [(((-0.1), 0.6, 0.4))]][0] for __l['l1l1lll1l_opy_'] in [((0.5, 0.5, (-0.3), 0.7))]][0] for __l['l1ll11ll1_opy_'] in [(((-0.4), 0.4, 0.1))]][0] for __l['l1ll11l1l_opy_'] in [(((-0.014727757546290993), (-0.415886141536534), (-0.9087277978469505), (-0.03218073733996252)))]][0] for __l['l1lll1l11_opy_'] in [(((-0.4), 0.4, 0.1))]][0])({}), 'l1l1ll1ll_opy_')]][0] for __g['l1l1lllll_opy_'], l1l1lllll_opy_.__name__ in [(lambda : (lambda __l: [[[[[[[[(__l['l1lll1l11_opy_'], __l['l1ll11l1l_opy_'], __l['l1ll11ll1_opy_'], __l['l1l1lll1l_opy_'], __l['l1ll1l11l_opy_'], __l['l1l1ll1l1_opy_'], __l['l1ll1111l_opy_'], __l['l1ll111l1_opy_']) for __l['l1ll111l1_opy_'] in [((0.0, 0.47, 0.0, 0.87))]][0] for __l['l1ll1111l_opy_'] in [(((-0.3), 0.3, 0.2))]][0] for __l['l1l1ll1l1_opy_'] in [((0.0, 0.47, 0.0, 0.87))]][0] for __l['l1ll1l11l_opy_'] in [((0.5, 0.1, 0.8))]][0] for __l['l1l1lll1l_opy_'] in [((0.0, 0.0, 1.0, 0.0))]][0] for __l['l1ll11ll1_opy_'] in [((0.3, 0.2, 1.0))]][0] for __l['l1ll11l1l_opy_'] in [((0.0, 0.0, 0.0, 1))]][0] for __l['l1lll1l11_opy_'] in [((0.3, 0.2, 1.0))]][0])({}), 'l1l1lllll_opy_')]][0] for __g['CartesianGrader'] in [(__mod.CartesianGrader)]][0])(__import__('sys_urdf_setup', __g, __g, ('CartesianGrader',), 0)) for __g['tf'] in [(__import__('tf', __g, __g))]][0] for __g['rospy'] in [(__import__('rospy', __g, __g))]][0] for __g['numpy'] in [(__import__('numpy', __g, __g))]][0])[1] for __g['sys'] in [(__import__('sys', __g, __g))]][0])(__import__('operator', level=0), __import__('builtins', level=0).__dict__['print'], globals())
