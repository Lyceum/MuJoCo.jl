const MAXUISECT = 10      # maximum number of sections
const MAXUIITEM = 80      # maximum number of items per section
const MAXUITEXT = 500     # maximum number of chars in edittext and other
const MAXUINAME = 40      # maximum number of chars in name
const MAXUIMULTI = 20      # maximum number of radio/select items in group
const MAXUIEDIT = 5       # maximum number of elements in edit list
const MAXUIRECT = 15      # maximum number of rectangles

# key codes matching GLFW (user must remap for other frameworks)
const KEY_ESCAPE = 256
const KEY_ENTER = 257
const KEY_TAB = 258
const KEY_BACKSPACE = 259
const KEY_INSERT = 260
const KEY_DELETE = 261
const KEY_RIGHT = 262
const KEY_LEFT = 263
const KEY_DOWN = 264
const KEY_UP = 265
const KEY_PAGE_UP = 266
const KEY_PAGE_DOWN = 267
const KEY_HOME = 268
const KEY_END = 269
const KEY_F1 = 290
const KEY_F2 = 291
const KEY_F3 = 292
const KEY_F4 = 293
const KEY_F5 = 294
const KEY_F6 = 295
const KEY_F7 = 296
const KEY_F8 = 297
const KEY_F9 = 298
const KEY_F10 = 299
const KEY_F11 = 300
const KEY_F12 = 301

@enum mjtButton::Cint begin        # mouse button
    BUTTON_NONE = 0          # no button
    BUTTON_LEFT              # left button
    BUTTON_RIGHT             # right button
    BUTTON_MIDDLE             # middle button
end

@enum mjtEvent::Cint begin         # mouse and keyboard event type
    EVENT_NONE = 0           # no event
    EVENT_MOVE               # mouse move
    EVENT_PRESS              # mouse button press
    EVENT_RELEASE            # mouse button release
    EVENT_SCROLL             # scroll
    EVENT_KEY                # key press
    EVENT_RESIZE              # resize
end

@enum mjtItem::Cint begin          # UI item type
    ITEM_END = -2            # end of definition list (not an item)
    ITEM_SECTION = -1        # section (not an item)
    ITEM_SEPARATOR = 0       # separator
    ITEM_STATIC              # static text
    ITEM_BUTTON              # button

   # the rest have data pointer
    ITEM_CHECKINT            # check box,value::Cint
    ITEM_CHECKBYTE           # check box, XXXByte value
    ITEM_RADIO               # radio group
    ITEM_SELECT              # selection box
    ITEM_SLIDERINT           # slider,value::Cint
    ITEM_SLIDERNUM           # slider, XXXNum value
    ITEM_EDITINT             # editable array,values::Cint
    ITEM_EDITNUM             # editable array, XXXNum values
    ITEM_EDITTXT             # editable text

    NITEM                     # number of item types
end

@uninitable mutable struct mjuiState               # mouse and keyboard state
   # constants set by user
    nrect::Cint                  # number of rectangles used
    rect::SVector{MAXUIRECT,mjrRect}  # rectangles (index 0: entire window)
    userdata::Ptr{Cvoid}             # pointer to user data (for callbacks)

   # event type
    type::Cint                   # (type XXXEvent)

   # mouse buttons
    left::Cint                   # is left button down
    right::Cint                  # is right button down
    middle::Cint                 # is middle button down
    doubleclick::Cint            # is last press a double click
    button::Cint                 # which button was pressed (XXXButton)
    buttontime::Cdouble          # time of last button press

   # mouse position
    x::Cdouble                   # x position
    y::Cdouble                   # y position
    dx::Cdouble                  # x displacement
    dy::Cdouble                  # y displacement
    sx::Cdouble                  # x scroll
    sy::Cdouble                  # y scroll

   # keyboard
    control::Cint                # is control down
    shift::Cint                  # is shift down
    alt::Cint                    # is alt down
    key::Cint                    # which key was pressed
    keytime::Cdouble             # time of last key press

   # rectangle ownership and dragging
    mouserect::Cint              # which rectangle contains mouse
    dragrect::Cint               # which rectangle is dragged with mouse
    dragbutton::Cint             # which button started drag (XXXButton)
end

@uninitable struct mjuiThemeSpacing        # UI visualization theme spacing
    total::Cint              # total width
    scroll::Cint             # scrollbar width
    label::Cint              # label width
    section::Cint            # section gap
    itemside::Cint           # item side gap
    itemmid::Cint            # item middle gap
    itemver::Cint            # item vertical gap
    texthor::Cint            # text horizontal gap
    textver::Cint            # text vertical gap
    linescroll::Cint         # number of pixels to scroll
    samples::Cint            # number of multisamples
end

@uninitable struct mjuiThemeColor          # UI visualization theme color
    master::SVector{3,Cfloat}            # master background
    thumb::SVector{3,Cfloat}             # scrollbar thumb
    secttitle::SVector{3,Cfloat}         # section title
    sectfont::SVector{3,Cfloat}          # section font
    sectsymbol::SVector{3,Cfloat}        # section symbol
    sectpane::SVector{3,Cfloat}          # section pane
    shortcut::SVector{3,Cfloat}          # shortcut background
    fontactive::SVector{3,Cfloat}        # font active
    fontinactive::SVector{3,Cfloat}      # font inactive
    decorinactive::SVector{3,Cfloat}     # decor inactive
    decorinactive2::SVector{3,Cfloat}    # inactive slider color 2
    button::SVector{3,Cfloat}            # button
    check::SVector{3,Cfloat}             # check
    radio::SVector{3,Cfloat}             # radio
    select::SVector{3,Cfloat}            # select
    select2::SVector{3,Cfloat}           # select pane
    slider::SVector{3,Cfloat}            # slider
    slider2::SVector{3,Cfloat}           # slider color 2
    edit::SVector{3,Cfloat}              # edit
    edit2::SVector{3,Cfloat}             # edit invalid
    cursor::SVector{3,Cfloat}            # edit cursor
end

# TODO: Julia can't currently represent C Union's
@uninitable struct mjuiItem                # UI item
   # common properties
    type::Cint                   # type (XXXItem)
    name::SVector{MAXUINAME,Cchar}     # name
    state::Cint                  # 0: disable, 1: enable, 2+: use predicate
    pdata::Ptr{Cvoid}                # data pointer (type-specific)
    sectionid::Cint              # id of section containing item
    itemid::Cint                 # id of item within section

   # JULIA cannot do union declaration as follows, so using largest size
    nelem::Cint
    _name::SVector{MAXUIMULTI,SVector{MAXUINAME,Cchar}}
   # type-specific properties
   #union
   #begin
   #    # check and button-related
   #    struct
   #    begin
   #       modifier::Cint       # 0: none, 1: control, 2: shift; 4: alt
   #       shortcut::Cint       # shortcut key; 0: undefined
   #    end single

   #    # static, radio and select-related
   #    struct
   #    begin
   #       nelem::Cint          # number of elements in group
   #        char name[mjMAXUIMULTI][mjMAXUINAME] # element names
   #    end multi

   #    # slider-related
   #    struct
   #    begin
   #        double range[2]    # slider range
   #        double divisions   # number of range divisions
   #    end slider

   #    # edit-related
   #    struct
   #    begin
   #       nelem::Cint          # number of elements in list
   #        double range[mjMAXUIEDIT][2] # element range (min>=max: ignore)
   #    end edit
   #end

   # internal
    rect::mjrRect                # rectangle occupied by item
end

@uninitable struct mjuiSection             # UI section
   # properties
    name::SVector{MAXUINAME,Cchar}     # name
    state::Cint                  # 0: closed, 1: open
    modifier::Cint               # 0: none, 1: control, 2: shift; 4: alt
    shortcut::Cint               # shortcut key; 0: undefined
    nitem::Cint                  # number of items in use
    item::SVector{MAXUIITEM,mjuiItem} # preallocated array of items

   # internal
    rtitle::mjrRect              # rectangle occupied by title
    rcontent::mjrRect            # rectangle occupied by content
end

@uninitable mutable struct mjUI                    # entire UI
   # constants set by user
    spacing::mjuiThemeSpacing    # UI theme spacing
    color::mjuiThemeColor        # UI theme color
    # NOTE: in C, typeof predicate is typedef int (*mjfItemEnable)(int category, void* data),
    # but C function pointers in Julia are just Ptr{Cvoid}
    predicate::Ptr{Cvoid}        # callback to set item state programmatically
    userdata::Ptr{Cvoid}         # pointer to user data (passed to predicate)
    rectid::Cint                 # index of this ui rectangle in mjuiState
    auxid::Cint                  # aux buffer index of this ui
    radiocol::Cint               # number of radio columns (0 defaults to 2)

   # UI sizes (framebuffer units)
    width::Cint                  # width
    height::Cint                 # current heigth
    maxheight::Cint              # height when all sections open
    scroll::Cint                 # scroll from top of UI

   # mouse focus
    mousesect::Cint              # 0: none, -1: scroll, otherwise 1+section
    mouseitem::Cint              # item within section
    mousehelp::Cint              # help button down: print shortcuts

   # keyboard focus and edit
    editsect::Cint               # 0: none, otherwise 1+section
    edititem::Cint               # item within section
    editcursor::Cint             # cursor position
    editscroll::Cint             # horizontal scroll
    edittext::SVector{MAXUITEXT,Cchar} # current text
    editchanged::Ptr{mjuiItem}      # pointer to changed edit in last mjui_event

   # sections
    nsect::Cint                  # number of sections in use
    sect::SVector{MAXUISECT,mjuiSection}  # preallocated array of sections
end

@uninitable mutable struct mjuiDef
    type::Cint                   # type (XXXItem); -1: section
    name::SVector{MAXUINAME,Cchar}     # name
    state::Cint                  # state
    pdata::Ptr{Cvoid}                # pointer to data
    other::SVector{MAXUITEXT,Cchar}     # string with type-specific properties
end


