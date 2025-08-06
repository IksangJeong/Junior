"""
데이터베이스 연습 #312
SQLite CRUD 실습
"""
import sqlite3

def setup_312():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_312 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_312_0', 462.23, 'A'), ('item_312_1', 563.21, 'C'), ('item_312_2', 808.85, 'C'), ('item_312_3', 31.55, 'B'), ('item_312_4', 818.29, 'C'), ('item_312_5', 225.72, 'A'), ('item_312_6', 54.84, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_312 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_312()
    cursor = conn.cursor()

    print(f"DB 연습 #312")
    cursor.execute('SELECT * FROM items_312')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_312 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
