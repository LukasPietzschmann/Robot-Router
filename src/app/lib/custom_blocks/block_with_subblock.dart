import 'package:flutter/material.dart';
import 'package:robot_router/block_view.dart';

class BlockWithSubblock extends Block {
  Block? subblock;

  @override
  String get name => 'Block with Subblock';

  @override
  BlockType get type => subblock?.type ?? BlockType.t_passive;

  @override
  BlockView<BlockWithSubblock> construct() =>
      BlockWithSubblockView(block: this, key: ObjectKey(this));
}

class BlockWithSubblockView extends BlockView<BlockWithSubblock> {
  const BlockWithSubblockView(
      {required BlockWithSubblock block, required Key key})
      : super(block: block, key: key);

  @override
  BlockViewState<BlockWithSubblockView> createState() =>
      BlockWithSubblockState();

  @override
  T accept<T>(BlockVisitor<T> visitor) {
    return visitor.visitBlockWithSubblock(this);
  }
}

class BlockWithSubblockState extends BlockViewState<BlockWithSubblockView> {
  BlockWithSubblockState();
  Block? subblock;

  @override
  void initState() {
    super.initState();
    subblock = widget.block.subblock;
  }

  @override
  Widget render() {
    return Row(
      children: <Widget>[
        const Text('This is a Block:'),
        Expanded(
          child: InkWell(
              child: Card(
                  color: Colors.grey,
                  shape: ContinuousRectangleBorder(
                    side: const BorderSide(color: Colors.black45, width: 1),
                    borderRadius: BorderRadius.circular(20),
                  ),
                  child: Padding(
                    padding: const EdgeInsets.all(10),
                    child: subblock?.construct() ??
                        const Text('Click the grey area to select a block',
                            style: TextStyle(color: Colors.white)),
                  )),
              onTap: () {
                selectBlockFromList(context).then((Block? block) {
                  setState(() {
                    subblock = block ?? subblock;
                  });
                  widget.block.subblock = block ?? subblock;
                });
              }),
        ),
      ],
    );
  }
}
