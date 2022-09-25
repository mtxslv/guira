import numpy as np
import pytest
from guira.curves import CubicPolynomials, Lemniscate

# Cubic Polynomials



# Lemniscate

@pytest.mark.parametrize('attributes_definition',
[
    ({'a_x': 1, 'a_y': 1, 'f_x': 1, 'f_y': 1}),
    ({'a_x': 1, 'a_y': 1})
])
def test_lemniscate_attributes(attributes_definition): # test if all attributes exist
    if len( attributes_definition.keys() ) > 2:
        lem = Lemniscate(a_x = attributes_definition['a_x'],
                         a_y = attributes_definition['a_y'],
                         f_x = attributes_definition['f_x'],
                         f_y = attributes_definition['f_y'])
    else:
        lem = Lemniscate(a_x = attributes_definition['a_x'],
                         a_y = attributes_definition['a_y'])
    attribute_sentence = hasattr(lem, 'a_x') and hasattr(lem, 'a_y') and hasattr(lem, 'f_x') and hasattr(lem, 'f_y') 
    assert attribute_sentence        

@pytest.mark.parametrize('attributes_definition',
[
    ({'a_x': -0.1, 'a_y': 2,    'f_x': 1,  'f_y': 1}),
    ({'a_x': 2.6,  'a_y': 2,    'f_x': 1,  'f_y': 1}),
    ({'a_x': 2,    'a_y': -0.1, 'f_x': 1,  'f_y': 1}),
    ({'a_x': 2,    'a_y': 2.6,  'f_x': 1,  'f_y': 1}),
    ({'a_x': 2,    'a_y': 2,    'f_x': -1, 'f_y': 2}),
    ({'a_x': 2,    'a_y': 2,    'f_x': 0,  'f_y': 2}),
    ({'a_x': 2,    'a_y': 2,    'f_x': 1, 'f_y': -1}),
    ({'a_x': 2,    'a_y': 2,    'f_x': 1,  'f_y': 0})
])
def test_lemniscate_wrong_interval(attributes_definition): 
    with pytest.raises(ValueError):
        lem = Lemniscate(a_x = attributes_definition['a_x'],
                         a_y = attributes_definition['a_y'],
                         f_x = attributes_definition['f_x'],
                         f_y = attributes_definition['f_y'])

def test_lemniscate_return_format(): # test the shape of what is returned
    lem = Lemniscate(2,2)
    ans = lem.get_point(2)
    assert_sentence = len(ans) == 4 and ans[0].shape == (2,) and ans[1].shape == (2,) and ans[2].shape == (2,) and isinstance(ans[3], float)
    assert assert_sentence

def test_lemniscate_return_values():
    lem = Lemniscate(2,2)
    ans = lem.get_point(2)
    expected_ans = (np.array([1.82709092, 1.48628965]),
                    np.array([-0.17037345,  0.56056955]),
                    np.array([-0.08014518, -0.26078384]),
                    1.865853970830244)
    assert_sentence = (expected_ans[0] == np.round(ans[0],8)).all() and (expected_ans[1] == np.round(ans[1],8)).all() and (expected_ans[2] == np.round(ans[2],8)).all() and expected_ans[3] == ans[3]
    assert assert_sentence