# This file was created automatically by SWIG 1.3.29.
# Don't modify this file, modify the SWIG interface instead.

import _rxtoolscpp
import new
new_instancemethod = new.instancemethod
def _swig_setattr_nondynamic(self,class_type,name,value,static=1):
    if (name == "thisown"): return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'PySwigObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name,None)
    if method: return method(self,value)
    if (not static) or hasattr(self,name):
        self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)

def _swig_setattr(self,class_type,name,value):
    return _swig_setattr_nondynamic(self,class_type,name,value,0)

def _swig_getattr(self,class_type,name):
    if (name == "thisown"): return self.this.own()
    method = class_type.__swig_getmethods__.get(name,None)
    if method: return method(self)
    raise AttributeError,name

def _swig_repr(self):
    try: strthis = "proxy of " + self.this.__repr__()
    except: strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

import types
try:
    _object = types.ObjectType
    _newclass = 1
except AttributeError:
    class _object : pass
    _newclass = 0
del types


def _swig_setattr_nondynamic_method(set):
    def set_attr(self,name,value):
        if (name == "thisown"): return self.this.own(value)
        if hasattr(self,name) or (name == "this"):
            set(self,name,value)
        else:
            raise AttributeError("You cannot add attributes to %s" % self)
    return set_attr


import wx._core
import wx._windows
class RosoutFilter(object):
    """Proxy of C++ RosoutFilter class"""
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    def __init__(self): raise AttributeError, "No constructor defined"
    __repr__ = _swig_repr
    __swig_destroy__ = _rxtoolscpp.delete_RosoutFilter
    __del__ = lambda self : None;
    def filter(*args, **kwargs):
        """filter(self, rosgraph_msgs::LogConstPtr ?) -> bool"""
        return _rxtoolscpp.RosoutFilter_filter(*args, **kwargs)

    def isValid(*args, **kwargs):
        """isValid(self) -> bool"""
        return _rxtoolscpp.RosoutFilter_isValid(*args, **kwargs)

    def isEnabled(*args, **kwargs):
        """isEnabled(self) -> bool"""
        return _rxtoolscpp.RosoutFilter_isEnabled(*args, **kwargs)

    def setEnabled(*args, **kwargs):
        """setEnabled(self, bool enabled)"""
        return _rxtoolscpp.RosoutFilter_setEnabled(*args, **kwargs)

    def getChangedSignal(*args, **kwargs):
        """getChangedSignal(self) -> RosoutFilterChangedSignal"""
        return _rxtoolscpp.RosoutFilter_getChangedSignal(*args, **kwargs)

_rxtoolscpp.RosoutFilter_swigregister(RosoutFilter)

class RosoutPanelBase(wx._windows.Panel):
    """Proxy of C++ RosoutPanelBase class"""
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    def __init__(self, *args, **kwargs): 
        """
        __init__(self, Window parent, int id=ID_ANY, Point pos=DefaultPosition, 
            Size size=wxSize( 800,600 ), long style=wxCLIP_CHILDREN|wxTAB_TRAVERSAL) -> RosoutPanelBase
        """
        _rxtoolscpp.RosoutPanelBase_swiginit(self,_rxtoolscpp.new_RosoutPanelBase(*args, **kwargs))
    __swig_destroy__ = _rxtoolscpp.delete_RosoutPanelBase
    __del__ = lambda self : None;
_rxtoolscpp.RosoutPanelBase_swigregister(RosoutPanelBase)

class RosoutSetupDialogBase(wx._windows.Dialog):
    """Proxy of C++ RosoutSetupDialogBase class"""
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    def __init__(self, *args, **kwargs): 
        """
        __init__(self, Window parent, int id=ID_ANY, String title=wxT("Rosout Panel Setup"), 
            Point pos=DefaultPosition, 
            Size size=wxSize( 331,214 ), long style=DEFAULT_DIALOG_STYLE) -> RosoutSetupDialogBase
        """
        _rxtoolscpp.RosoutSetupDialogBase_swiginit(self,_rxtoolscpp.new_RosoutSetupDialogBase(*args, **kwargs))
    __swig_destroy__ = _rxtoolscpp.delete_RosoutSetupDialogBase
    __del__ = lambda self : None;
_rxtoolscpp.RosoutSetupDialogBase_swigregister(RosoutSetupDialogBase)

class TextboxDialog(wx._windows.Dialog):
    """Proxy of C++ TextboxDialog class"""
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    text_ = property(_rxtoolscpp.TextboxDialog_text__get, _rxtoolscpp.TextboxDialog_text__set)
    def __init__(self, *args, **kwargs): 
        """
        __init__(self, Window parent, int id=ID_ANY, String title=wxEmptyString, 
            Point pos=DefaultPosition, Size size=wxSize( 644,362 ), 
            long style=wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER) -> TextboxDialog
        """
        _rxtoolscpp.TextboxDialog_swiginit(self,_rxtoolscpp.new_TextboxDialog(*args, **kwargs))
    __swig_destroy__ = _rxtoolscpp.delete_TextboxDialog
    __del__ = lambda self : None;
_rxtoolscpp.TextboxDialog_swigregister(TextboxDialog)

class LoggerLevelPanelBase(wx._windows.Panel):
    """Proxy of C++ LoggerLevelPanelBase class"""
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    def __init__(self, *args, **kwargs): 
        """
        __init__(self, Window parent, int id=ID_ANY, Point pos=DefaultPosition, 
            Size size=wxSize( 500,300 ), long style=TAB_TRAVERSAL) -> LoggerLevelPanelBase
        """
        _rxtoolscpp.LoggerLevelPanelBase_swiginit(self,_rxtoolscpp.new_LoggerLevelPanelBase(*args, **kwargs))
    __swig_destroy__ = _rxtoolscpp.delete_LoggerLevelPanelBase
    __del__ = lambda self : None;
_rxtoolscpp.LoggerLevelPanelBase_swigregister(LoggerLevelPanelBase)

class RosoutTextFilterControlBase(wx._windows.Panel):
    """Proxy of C++ RosoutTextFilterControlBase class"""
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    def __init__(self, *args, **kwargs): 
        """
        __init__(self, Window parent, int id=ID_ANY, Point pos=DefaultPosition, 
            Size size=wxSize( 750,42 ), long style=TAB_TRAVERSAL) -> RosoutTextFilterControlBase
        """
        _rxtoolscpp.RosoutTextFilterControlBase_swiginit(self,_rxtoolscpp.new_RosoutTextFilterControlBase(*args, **kwargs))
    __swig_destroy__ = _rxtoolscpp.delete_RosoutTextFilterControlBase
    __del__ = lambda self : None;
_rxtoolscpp.RosoutTextFilterControlBase_swigregister(RosoutTextFilterControlBase)

class RosoutSeverityFilterControlBase(wx._windows.Panel):
    """Proxy of C++ RosoutSeverityFilterControlBase class"""
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    def __init__(self, *args, **kwargs): 
        """
        __init__(self, Window parent, int id=ID_ANY, Point pos=DefaultPosition, 
            Size size=wxSize( 750,42 ), long style=TAB_TRAVERSAL) -> RosoutSeverityFilterControlBase
        """
        _rxtoolscpp.RosoutSeverityFilterControlBase_swiginit(self,_rxtoolscpp.new_RosoutSeverityFilterControlBase(*args, **kwargs))
    __swig_destroy__ = _rxtoolscpp.delete_RosoutSeverityFilterControlBase
    __del__ = lambda self : None;
_rxtoolscpp.RosoutSeverityFilterControlBase_swigregister(RosoutSeverityFilterControlBase)

class RosoutFrame(wx._windows.Frame):
    """Proxy of C++ RosoutFrame class"""
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    rosout_panel_ = property(_rxtoolscpp.RosoutFrame_rosout_panel__get, _rxtoolscpp.RosoutFrame_rosout_panel__set)
    def __init__(self, *args, **kwargs): 
        """
        __init__(self, Window parent, int id=ID_ANY, String title=wxT("rxconsole"), 
            Point pos=DefaultPosition, Size size=wxSize( 800,600 ), 
            long style=wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL) -> RosoutFrame
        """
        _rxtoolscpp.RosoutFrame_swiginit(self,_rxtoolscpp.new_RosoutFrame(*args, **kwargs))
    __swig_destroy__ = _rxtoolscpp.delete_RosoutFrame
    __del__ = lambda self : None;
_rxtoolscpp.RosoutFrame_swigregister(RosoutFrame)

class LoggerLevelFrame(wx._windows.Frame):
    """Proxy of C++ LoggerLevelFrame class"""
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    logger_panel_ = property(_rxtoolscpp.LoggerLevelFrame_logger_panel__get, _rxtoolscpp.LoggerLevelFrame_logger_panel__set)
    def __init__(self, *args, **kwargs): 
        """
        __init__(self, Window parent, int id=ID_ANY, String title=wxT("rxloggerlevel"), 
            Point pos=DefaultPosition, Size size=wxSize( 800,200 ), 
            long style=wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL) -> LoggerLevelFrame
        """
        _rxtoolscpp.LoggerLevelFrame_swiginit(self,_rxtoolscpp.new_LoggerLevelFrame(*args, **kwargs))
    __swig_destroy__ = _rxtoolscpp.delete_LoggerLevelFrame
    __del__ = lambda self : None;
_rxtoolscpp.LoggerLevelFrame_swigregister(LoggerLevelFrame)

class RosoutMessageSummary(object):
    """Proxy of C++ RosoutMessageSummary class"""
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    def __init__(self, *args, **kwargs): 
        """__init__(self) -> RosoutMessageSummary"""
        _rxtoolscpp.RosoutMessageSummary_swiginit(self,_rxtoolscpp.new_RosoutMessageSummary(*args, **kwargs))
    __swig_destroy__ = _rxtoolscpp.delete_RosoutMessageSummary
    __del__ = lambda self : None;
    debug = property(_rxtoolscpp.RosoutMessageSummary_debug_get, _rxtoolscpp.RosoutMessageSummary_debug_set)
    info = property(_rxtoolscpp.RosoutMessageSummary_info_get, _rxtoolscpp.RosoutMessageSummary_info_set)
    warn = property(_rxtoolscpp.RosoutMessageSummary_warn_get, _rxtoolscpp.RosoutMessageSummary_warn_set)
    error = property(_rxtoolscpp.RosoutMessageSummary_error_get, _rxtoolscpp.RosoutMessageSummary_error_set)
    fatal = property(_rxtoolscpp.RosoutMessageSummary_fatal_get, _rxtoolscpp.RosoutMessageSummary_fatal_set)
_rxtoolscpp.RosoutMessageSummary_swigregister(RosoutMessageSummary)

class RosoutPanel(RosoutPanelBase):
    """Proxy of C++ RosoutPanel class"""
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    def __init__(self, *args, **kwargs): 
        """
        __init__(self, Window parent, int id=ID_ANY, Point pos=DefaultPosition, 
            Size size=DefaultSize, int style=TAB_TRAVERSAL) -> RosoutPanel
        """
        _rxtoolscpp.RosoutPanel_swiginit(self,_rxtoolscpp.new_RosoutPanel(*args, **kwargs))
        self._setOORInfo(self)

    __swig_destroy__ = _rxtoolscpp.delete_RosoutPanel
    __del__ = lambda self : None;
    def setEnabled(*args, **kwargs):
        """setEnabled(self, bool enabled)"""
        return _rxtoolscpp.RosoutPanel_setEnabled(*args, **kwargs)

    def setTopic(*args, **kwargs):
        """setTopic(self, string topic)"""
        return _rxtoolscpp.RosoutPanel_setTopic(*args, **kwargs)

    def clear(*args, **kwargs):
        """clear(self)"""
        return _rxtoolscpp.RosoutPanel_clear(*args, **kwargs)

    def setBufferSize(*args, **kwargs):
        """setBufferSize(self, uint32_t size)"""
        return _rxtoolscpp.RosoutPanel_setBufferSize(*args, **kwargs)

    def getMessageSummary(*args, **kwargs):
        """getMessageSummary(self, double duration) -> RosoutMessageSummary"""
        return _rxtoolscpp.RosoutPanel_getMessageSummary(*args, **kwargs)

    def getMessageByIndex(*args, **kwargs):
        """getMessageByIndex(self, uint32_t index) -> rosgraph_msgs::LogConstPtr"""
        return _rxtoolscpp.RosoutPanel_getMessageByIndex(*args, **kwargs)

    def createTextFilter(*args, **kwargs):
        """createTextFilter(self) -> RosoutTextFilterPtr"""
        return _rxtoolscpp.RosoutPanel_createTextFilter(*args, **kwargs)

    def createNewFrame(*args, **kwargs):
        """createNewFrame(self) -> RosoutFrame"""
        return _rxtoolscpp.RosoutPanel_createNewFrame(*args, **kwargs)

    def clearFilters(*args, **kwargs):
        """clearFilters(self)"""
        return _rxtoolscpp.RosoutPanel_clearFilters(*args, **kwargs)

    def refilter(*args, **kwargs):
        """refilter(self)"""
        return _rxtoolscpp.RosoutPanel_refilter(*args, **kwargs)

_rxtoolscpp.RosoutPanel_swigregister(RosoutPanel)


def initRoscpp(*args, **kwargs):
  """initRoscpp(string node_name, bool anonymous)"""
  return _rxtoolscpp.initRoscpp(*args, **kwargs)


